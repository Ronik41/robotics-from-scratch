"""Manual acceptance observer: press i then k in the browser teleop terminal."""
import json
import math
from pathlib import Path
import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

rclpy.init()
node = Node('keyboard_acceptance_observer')
commands, poses = [], []

def on_command(msg):
    commands.append({'linear_x': msg.linear.x, 'angular_z': msg.angular.z, 'wall_time': time.time()})

node.create_subscription(Twist, '/cmd_vel', on_command, 10)
node.create_subscription(Odometry, '/odom', lambda m: poses.append(m), 20)
result = {'status': 'FAIL'}
try:
    deadline = time.monotonic()+90
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if (any(m['linear_x'] > 0 for m in commands) and commands[-1]['linear_x'] == 0
                and commands[-1]['angular_z'] == 0 and time.time()-commands[-1]['wall_time'] > 1):
            break
    else:
        raise TimeoutError('Expected browser keyboard forward followed by explicit stop')
    start, end = poses[0].pose.pose.position, poses[-1].pose.pose.position
    distance = math.hypot(end.x-start.x, end.y-start.y)
    assert distance > 0.02, f'Keyboard did not move odometry: {distance}'
    assert abs(poses[-1].twist.twist.linear.x) < 0.01, 'Keyboard stop did not stop rover'
    result = {'status': 'PASS', 'commands': commands, 'distance_m': distance,
              'final_linear_x': poses[-1].twist.twist.linear.x, 'odometry_messages': len(poses)}
finally:
    Path(sys.argv[1]).write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result))
    node.destroy_node()
    rclpy.shutdown()
