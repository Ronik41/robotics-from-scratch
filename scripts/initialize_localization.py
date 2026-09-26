"""Supply an operator's approximate base_drive (axle) pose in the map."""
import argparse
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from rosgraph_msgs.msg import Clock


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--x', type=float, default=.25)
    p.add_argument('--y', type=float, default=-.20)
    p.add_argument('--yaw', type=float, default=.20)
    args = p.parse_args()
    if not all(math.isfinite(x) for x in (args.x, args.y, args.yaw)):
        p.error('Pose values must be finite')
    rclpy.init(); node = Node('operator_initial_pose')
    clock = []
    node.create_subscription(Clock, '/clock', lambda m: clock.append(m.clock), qos_profile_sensor_data)
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
    try:
        end = time.monotonic() + 60
        while True:
            endpoints = node.get_subscriptions_info_by_topic('/initialpose')
            # A recorder may discover us before AMCL. Acknowledgement by that
            # observer alone does not initialize the filter. Wait for AMCL too.
            if clock and any(i.node_name == 'amcl' for i in endpoints) and pub.get_subscription_count() >= len(endpoints):
                break
            if time.monotonic() > end:
                raise TimeoutError('AMCL /initialpose or simulation clock unavailable')
            rclpy.spin_once(node, timeout_sec=.05)
        msg = PoseWithCovarianceStamped()
        msg.header.stamp, msg.header.frame_id = clock[-1], 'map'
        msg.pose.pose.position.x, msg.pose.pose.position.y = args.x, args.y
        msg.pose.pose.orientation.z = math.sin(args.yaw/2)
        msg.pose.pose.orientation.w = math.cos(args.yaw/2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = .25**2
        msg.pose.covariance[35] = .30**2
        pub.publish(msg)
        if not pub.wait_for_all_acked(Duration(seconds=3)):
            raise TimeoutError('Initial pose was not acknowledged')
        print(f'Approximate base_drive pose in map: ({args.x}, {args.y}, {args.yaw}); sigma=(.25m,.25m,.30rad)')
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
