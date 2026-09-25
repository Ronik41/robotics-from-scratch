"""Wheel-only odometry and the single odom -> base_link TF authority."""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from kinematics import WheelOdometry


class OdometryNode(Node):
    def __init__(self):
        super().__init__('wheel_odometry')
        radius = self.declare_parameter('wheel_radius', 0.14).value
        separation = self.declare_parameter('wheel_separation', 0.48).value
        axle_x = self.declare_parameter('axle_x', 0.14).value
        self.model = WheelOdometry(radius, separation, axle_x)
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)
        self.last_published = None

    def on_joints(self, msg):
        try:
            left = msg.position[msg.name.index('left_wheel_joint')]
            right = msg.position[msg.name.index('right_wheel_joint')]
        except (ValueError, IndexError):
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        state = self.model.update(stamp, left, right)
        if state is None:
            return
        # Some Harmonic builds publish joint feedback at every physics step.
        # Integrate every sample, publish at at most 50 Hz in simulation time.
        if self.last_published is not None and stamp - self.last_published < 0.019999:
            return
        self.last_published = stamp
        x, y, yaw, vx, vy, wz = state
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        odom.pose.pose.orientation.z = math.sin(yaw / 2)
        odom.pose.pose.orientation.w = math.cos(yaw / 2)
        odom.twist.twist.linear.x, odom.twist.twist.linear.y = vx, vy
        odom.twist.twist.angular.z = wz
        # Nonzero placeholders, NOT calibrated uncertainty or a slip/noise model.
        for i, variance in enumerate([0.01, 0.01, 1e6, 1e6, 1e6, 0.02]):
            odom.pose.covariance[7*i] = variance
            odom.twist.covariance[7*i] = variance
        self.pub.publish(odom)
        tf = TransformStamped()
        tf.header, tf.child_frame_id = odom.header, odom.child_frame_id
        tf.transform.translation.x, tf.transform.translation.y = x, y
        tf.transform.rotation = odom.pose.pose.orientation
        self.tf.sendTransform(tf)


def main():
    rclpy.init()
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
