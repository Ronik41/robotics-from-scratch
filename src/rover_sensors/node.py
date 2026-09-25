"""Simulation driver boundary: raw physics -> imperfect ROS measurements.

Native IMU attitude never enters ROS. No pose, TF, odometry, or control output.
"""
from collections import deque
from pathlib import Path
import queue
import random
import math
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from rover_interfaces.msg import WheelEncoders
from gz.transport13 import Node as GzNode
from gz.msgs10.imu_pb2 import IMU
from models import encoder_count

ROOT = Path(__file__).resolve().parents[2]


class Sensors(Node):
    def __init__(self):
        super().__init__('sim_sensor_driver')
        self.cfg = yaml.safe_load((ROOT/'simulation/config/sensors.yaml').read_text())
        self.rng = random.Random(self.cfg['seed'])
        self.pending = {'imu': deque(), 'encoders': deque()}
        self.raw = queue.Queue(maxsize=1000)
        self.last_imu = self.last_encoder = 0
        self.origin = None
        self.index = 0
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', qos_profile_sensor_data)
        self.encoder_pub = self.create_publisher(WheelEncoders, '/wheel/encoders', qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)
        self.gz = GzNode()
        self.gz.subscribe(IMU, '/sensors/imu_native', self.on_native)
        self.create_timer(0.002, self.flush)

    def on_native(self, message):
        # Transport callbacks run on another thread; all RNG/ROS work stays in executor.
        # Extract only measurable fields, explicitly dropping privileged orientation.
        sample = (message.header.stamp.sec*10**9+message.header.stamp.nsec,
                  [getattr(message.angular_velocity, k) for k in 'xyz'],
                  [getattr(message.linear_acceleration, k) for k in 'xyz'])
        try:
            self.raw.put_nowait(sample)
        except queue.Full:
            self.get_logger().error('Native IMU queue overflow; dropping sample')

    def on_joints(self, message):
        ns = message.header.stamp.sec*10**9+message.header.stamp.nanosec
        values = dict(zip(message.name, message.position))
        names = ['left_wheel_joint', 'right_wheel_joint']
        if ns <= self.last_encoder or any(n not in values or not math.isfinite(values[n]) for n in names):
            return
        angles = [values[n] for n in names]
        if self.origin is None:
            self.origin = angles
        msg = WheelEncoders()
        msg.header.stamp = message.header.stamp
        msg.header.frame_id = 'base_link'
        msg.sample_index = self.index
        dt = ns-self.last_encoder if self.last_encoder else 0
        msg.sample_period.sec, msg.sample_period.nanosec = divmod(dt, 10**9)
        msg.left_joint_name, msg.right_joint_name = names
        msg.counts_per_revolution = self.cfg['encoder_counts_per_revolution']
        msg.left_count, msg.right_count = [encoder_count(a, o, msg.counts_per_revolution)
                                         for a, o in zip(angles, self.origin)]
        self.pending['encoders'].append((ns+round(self.cfg['encoder_delay_s']*1e9), msg))
        self.last_encoder = ns
        self.index += 1

    def flush(self):
        while not self.raw.empty():
            ns, gyro, accel = self.raw.get_nowait()
            if ns <= self.last_imu or not all(math.isfinite(v) for v in gyro+accel):
                continue
            msg = Imu()
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 10**9)
            msg.header.frame_id = 'imu_link'
            msg.orientation.w = 0.0  # explicit invalid placeholder, not identity attitude
            msg.orientation_covariance[0] = -1.0  # no attitude estimate
            for vector, data, kind in [(msg.angular_velocity, gyro, 'gyro'),
                                       (msg.linear_acceleration, accel, 'accel')]:
                sigma = self.cfg[f'imu_{kind}_stddev']
                for axis, value, bias in zip('xyz', data, self.cfg[f'imu_{kind}_bias']):
                    setattr(vector, axis, value+bias+self.rng.gauss(0, sigma))
            # White measurement-noise covariance; fixed bias is separately documented.
            for i in (0, 4, 8):
                msg.angular_velocity_covariance[i] = self.cfg['imu_gyro_stddev']**2
                msg.linear_acceleration_covariance[i] = self.cfg['imu_accel_stddev']**2
            self.pending['imu'].append((ns+round(self.cfg['imu_delay_s']*1e9), msg))
            self.last_imu = ns
        now = self.get_clock().now().nanoseconds
        for key, pub in [('imu', self.imu_pub), ('encoders', self.encoder_pub)]:
            pending = self.pending[key]
            while pending and pending[0][0] <= now:
                pub.publish(pending.popleft()[1])


def main():
    rclpy.init()
    node = Sensors()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
