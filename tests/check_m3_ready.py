"""Non-moving ROS readiness probe for the desktop launcher."""
import json
from pathlib import Path
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener

rclpy.init()
node = Node('m3_readiness')
buffer = Buffer()
listener = TransformListener(buffer, node)
state = []
node.create_subscription(Odometry, '/odom', lambda m: state.append(m), 10)
result = {'status': 'FAIL'}
try:
    end = time.monotonic()+45
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if len(state) >= 3 and all(buffer.can_transform('odom', frame, Time()) for frame in
                                  ['base_link','left_wheel','right_wheel','rear_support','parcel_tray']):
            if state[-1].header.stamp != state[0].header.stamp:
                result = {'status': 'PASS', 'odometry_messages': len(state), 'frames': buffer.all_frames_as_yaml()}
                break
    else:
        raise TimeoutError('Live odometry / complete TF tree not ready')
finally:
    Path(sys.argv[1]).write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result))
    node.destroy_node()
    rclpy.shutdown()
