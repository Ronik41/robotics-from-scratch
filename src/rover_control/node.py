"""ROS-facing simulated firmware; all feedback comes from M4 encoder counts."""
from dataclasses import asdict
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from rover_interfaces.msg import WheelEncoders
from gz.transport13 import Node as GzNode
from gz.msgs10.double_v_pb2 import Double_V
from gz.msgs10.boolean_pb2 import Boolean
from core import Controller, Limits


def seconds(stamp):
    return stamp.sec + stamp.nanosec*1e-9


class Firmware(Node):
    def __init__(self):
        super().__init__('sim_firmware')
        radius = self.declare_parameter('wheel_radius', .14).value
        track = self.declare_parameter('wheel_separation', .48).value
        self.core = Controller(Limits(radius=radius, track=track))
        self.gz = GzNode()
        self.motor = self.gz.advertise('/motor/effort', Double_V)
        self.state_pub = self.create_publisher(String, '/firmware/state', 10)
        self.create_subscription(TwistStamped, '/cmd_vel', self.command, 1)
        self.create_subscription(WheelEncoders, '/wheel/encoders', self.encoder, qos_profile_sensor_data)
        self.create_service(SetBool, '/safety/estop', self.estop)
        self.create_service(Trigger, '/safety/reset', self.reset)
        self.last_sim = 0.
        self.last_advance = time.monotonic()
        self.last_sent = -1.
        self.last_report = 0.
        self.last_state = None
        self.create_timer(.01, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('Firmware limits: '+json.dumps(asdict(self.core.cfg)))

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def command(self, m):
        t = m.twist
        self.core.command(seconds(m.header.stamp),
                          [t.linear.x,t.linear.y,t.linear.z,t.angular.x,t.angular.y,t.angular.z],
                          m.header.frame_id, self.now(), time.monotonic())
        self.tick()

    def encoder(self, m):
        metadata = (m.header.frame_id == 'base_link' and m.left_joint_name == 'left_wheel_joint'
                    and m.right_joint_name == 'right_wheel_joint')
        self.core.encoder(seconds(m.header.stamp), m.sample_index, [m.left_count,m.right_count],
                          m.counts_per_revolution, seconds(m.sample_period), self.now(), time.monotonic(), metadata)

    def estop(self, req, res):
        self.core.set_estop(req.data)
        self.tick()
        res.success = True
        res.message = 'E-stop latched' if req.data else 'Input released; latch requires /safety/reset'
        return res

    def reset(self, req, res):
        del req
        now, wall = self.now(), time.monotonic()
        if not self.core.can_reset(now, wall):
            res.success = False
            res.message = 'Release e-stop, send fresh zero command, wait for fresh stationary encoders'
            return res
        ok, reply = self.gz.request('/motor/reset', Boolean(data=True), Boolean, Boolean, 500)
        res.success = bool(ok and reply.data)
        if res.success:
            res.success = self.core.reset(self.now(), time.monotonic())
        res.message = 'Reset; send a new command to move' if res.success else 'Motor driver refused reset'
        return res

    def tick(self):
        now, wall = self.now(), time.monotonic()
        if now < self.last_sim:
            self.core.set_estop(True)
        if now != self.last_sim:
            self.last_sim, self.last_advance = now, wall
        state = self.core.snapshot(now, wall)
        if wall-self.last_advance > 1:
            self.core.inhibit('CLOCK_STALLED')
            state = self.core.snapshot(now, wall)
        # Never renew driver lease with an unchanged simulation stamp.
        if now > self.last_sent:
            mode = 2 if self.core.estop else 1 if state['state'] == 'ACTIVE' else 0
            self.motor.publish(Double_V(data=[now, *self.core.effort, float(mode)]))
            self.last_sent = now
        if state['state'] != self.last_state:
            self.get_logger().info('Safety state: '+state['state'])
            self.last_state = state['state']
        if wall-self.last_report >= .05:
            state['sim_time'] = now
            self.state_pub.publish(String(data=json.dumps(state, allow_nan=False)))
            self.last_report = wall


def main():
    rclpy.init()
    node = Firmware()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
