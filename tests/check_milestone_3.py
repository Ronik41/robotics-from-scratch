"""Real ROS/Gazebo acceptance. Simulator pose exists ONLY in this test process."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node as RosNode
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from rosgraph_msgs.msg import Clock
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from rover_sim import ROOT, PREFIX, SCENARIO, save_json


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def require(condition, message):
    if not condition:
        raise AssertionError(message)


class Probe(RosNode):
    def __init__(self):
        super().__init__('milestone_3_acceptance')
        self.odom = self.joints = None
        self.clock = 0.0
        self.count = 0
        self.stamps = []
        self.edges = {}
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 100)
        self.create_subscription(JointState, '/joint_states', lambda m: setattr(self, 'joints', m), qos_profile_sensor_data)
        self.create_subscription(Clock, '/clock', lambda m: setattr(self, 'clock', m.clock.sec+m.clock.nanosec*1e-9), qos_profile_sensor_data)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 100)
        self.create_subscription(TFMessage, '/tf_static', self.on_tf,
                                 QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.truth = None
        self.truth_lock = threading.Lock()
        self.gz = GzNode()
        self.gz.subscribe(Pose_V, PREFIX+'/pose/info', self.on_truth)

    def on_odom(self, message):
        self.odom = message
        self.count += 1
        self.stamps.append(message.header.stamp.sec+message.header.stamp.nanosec*1e-9)

    def on_tf(self, message):
        for tf in message.transforms:
            self.edges.setdefault(tf.child_frame_id, set()).add(tf.header.frame_id)

    def on_truth(self, message):
        for pose in message.pose:
            if pose.name == SCENARIO['model']:
                with self.truth_lock:
                    self.truth = [pose.position.x, pose.position.y, yaw(pose.orientation),
                                  message.header.stamp.sec+message.header.stamp.nsec*1e-9]

    def command(self, v=0., w=0.):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.pub.publish(msg)

    def wait(self, predicate, timeout=45):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate():
                return
        raise TimeoutError('ROS/Gazebo observation deadline')

    def drive(self, seconds, v=0., w=0.):
        start = self.clock
        deadline = time.monotonic()+max(30, seconds*8)
        last_send = 0
        while self.clock-start < seconds:
            require(time.monotonic() < deadline, 'Simulation clock stalled')
            if time.monotonic()-last_send >= 0.05:
                self.command(v, w)
                last_send = time.monotonic()
            rclpy.spin_once(self, timeout_sec=0.01)

    def sample(self):
        # Drain until odom, TF and truth describe nearly the same simulation time.
        self.wait(lambda: self.odom is not None and self.truth is not None and
                  abs(self.stamps[-1]-self.truth[3]) < 0.08 and
                  all(self.buffer.can_transform('odom', frame, Time.from_msg(self.odom.header.stamp))
                      for frame in ['base_link','left_wheel','right_wheel','parcel_tray','rear_support']), 15)
        m = self.odom
        p, q, t = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist
        require(m.header.frame_id == 'odom' and m.child_frame_id == 'base_link', 'Odometry frame contract')
        require(abs(sum(getattr(q, k)**2 for k in 'xyzw')-1) < 1e-6, 'Quaternion not normalized')
        require(all(math.isfinite(v) for v in [p.x,p.y,p.z,t.linear.x,t.linear.y,t.angular.z]), 'Nonfinite odometry')
        require(m.pose.covariance[0] > 0, 'Unknown covariance represented as perfect certainty')
        tf = self.buffer.lookup_transform('odom', 'base_link', Time.from_msg(m.header.stamp))
        require(math.hypot(tf.transform.translation.x-p.x, tf.transform.translation.y-p.y) < 1e-8, 'TF and odom disagree')
        require(abs(angle(yaw(tf.transform.rotation)-yaw(q))) < 1e-8, 'TF orientation disagrees')
        for frame in ['left_wheel', 'right_wheel', 'parcel_tray', 'rear_support']:
            self.buffer.lookup_transform('odom', frame, Time.from_msg(m.header.stamp))
        joints = dict(zip(self.joints.name, self.joints.position))
        for side, sign in [('left', 1), ('right', -1)]:
            wheel = self.buffer.lookup_transform('base_link', side+'_wheel', Time.from_msg(m.header.stamp)).transform
            require(math.dist([wheel.translation.x, wheel.translation.y, wheel.translation.z],
                              [0.14, sign*0.24, -0.10]) < 1e-8, 'Wheel TF origin mismatch')
            # Compare modulo 2pi because equivalent quaternion signs may differ.
            rotation = 2*math.atan2(wheel.rotation.y, wheel.rotation.w)
            require(abs(angle(rotation-joints[side+'_wheel_joint'])) < 0.10, 'Wheel TF does not track joint rotation')
        with self.truth_lock:
            truth = list(self.truth)
        return {'sim_time': self.stamps[-1], 'odom': [p.x,p.y,yaw(q)], 'twist': [t.linear.x,t.linear.y,t.angular.z],
                'joints': joints, 'truth': truth}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--evidence', type=Path, required=True)
    parser.add_argument('--existing', action='store_true', help='Exercise the current desktop instead of starting a server')
    args = parser.parse_args()
    args.evidence.mkdir(parents=True, exist_ok=True)
    process = probe = log = None
    result = {'status': 'FAIL', 'phases': {}}
    rclpy.init()
    try:
        if not args.existing:
            log = (args.evidence/'launch.log').open('w')
            process = subprocess.Popen(['python3', str(ROOT/'scripts/rover_sim.py'), '--drive', '--evidence', str(args.evidence/'runtime')],
                                       stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        probe = Probe()
        probe.wait(lambda: probe.odom is not None and probe.joints is not None and probe.truth is not None and probe.pub.get_subscription_count() == 1)
        probe.drive(0.5)
        initial = probe.sample()
        if not args.existing:
            require(math.hypot(*initial['odom'][:2]) < 0.01, 'Fresh odom must start locally at zero, not world spawn coordinates')
        phases = [('forward', 2.5, 0.2, 0.), ('reverse', 1.0, -0.2, 0.),
                  ('left', 2.0, 0., 0.5), ('right', 1.0, 0., -0.5), ('arc', 2.0, 0.2, 0.4)]
        previous = initial
        for name, duration, v, w in phases:
            probe.drive(duration, v, w)
            moving = probe.sample()
            probe.drive(0.4)  # Explicit zero command; NOT a timeout/watchdog claim.
            current = probe.sample()
            dx, dy = (current['odom'][i]-previous['odom'][i] for i in (0,1))
            forward = dx*math.cos(previous['odom'][2])+dy*math.sin(previous['odom'][2])
            turn = angle(current['odom'][2]-previous['odom'][2])
            dl = current['joints']['left_wheel_joint']-previous['joints']['left_wheel_joint']
            dr = current['joints']['right_wheel_joint']-previous['joints']['right_wheel_joint']
            if v != 0:
                require(forward*v > 0 and abs(forward) > abs(v)*duration*0.65, f'{name}: did not translate correctly')
                require(dl*v > 0 and dr*v > 0, f'{name}: actual wheels did not rotate correctly')
            if w != 0:
                require(turn*w > 0 and abs(turn) > abs(w)*duration*0.65, f'{name}: did not turn correctly')
                if v == 0:
                    require(dl*w < 0 and dr*w > 0, f'{name}: wheel signs incorrect')
            tx = current['truth'][0]-initial['truth'][0]
            ty = current['truth'][1]-initial['truth'][1]
            heading = initial['truth'][2]
            expected = [tx*math.cos(heading)+ty*math.sin(heading), -tx*math.sin(heading)+ty*math.cos(heading)]
            ox = current['odom'][0]-initial['odom'][0]
            oy = current['odom'][1]-initial['odom'][1]
            oh = initial['odom'][2]
            observed = [ox*math.cos(oh)+oy*math.sin(oh), -ox*math.sin(oh)+oy*math.cos(oh)]
            position_error = math.dist(expected, observed)
            yaw_error = abs(angle(current['odom'][2]-initial['odom'][2]-current['truth'][2]+initial['truth'][2]))
            result['phases'][name] = {'moving': moving, 'stopped': current,
                                      'position_error_m': position_error, 'yaw_error_rad': yaw_error}
            # Finite-width contacts slip during turns. This is a nominal sanity
            # budget, not a claim of ground-truth odometry or localization accuracy.
            require(position_error < 0.10 and yaw_error < 0.20, f'{name}: wheel/Gazebo disagreement {position_error=}, {yaw_error=}')
            truth_turn = angle(current['truth'][2]-previous['truth'][2])
            truth_forward = ((current['truth'][0]-previous['truth'][0])*math.cos(previous['truth'][2])+
                             (current['truth'][1]-previous['truth'][1])*math.sin(previous['truth'][2]))
            if v != 0:
                require(truth_forward*v > 0 and abs(truth_forward) > abs(v)*duration*0.65,
                        f'{name}: physical rover did not translate correctly')
            if w != 0:
                require(truth_turn*w > 0 and abs(truth_turn) > abs(w)*duration*0.65,
                        f'{name}: physical rover did not turn correctly')
            previous = current
        probe.drive(1.0)
        final = probe.sample()
        require(math.dist(previous['odom'][:2], final['odom'][:2]) < 0.005, 'Zero command did not stop odometry')
        require(math.dist(previous['truth'][:2], final['truth'][:2]) < 0.005, 'Zero command did not stop Gazebo rover')
        require(max(abs(v) for v in final['twist']) < 0.01, 'Stopped twist nonzero')
        require(all(b>a for a,b in zip(probe.stamps, probe.stamps[1:])), 'Odometry stamps not increasing')
        rate = (len(probe.stamps)-1)/(probe.stamps[-1]-probe.stamps[0])
        require(35 < rate < 55, f'Unexpected odometry rate: {rate}')
        expected_edges = {'base_link': {'odom'}, 'left_wheel': {'base_link'}, 'right_wheel': {'base_link'},
                          'rear_support': {'base_link'}, 'parcel_tray': {'base_link'}}
        require(probe.edges == expected_edges, f'Invalid/extra/duplicate-parent TF frames: {probe.edges}')
        require(probe.count_publishers('/odom') == 1, 'Multiple odometry authorities')
        tf_publishers = sorted(i.node_name for i in probe.get_publishers_info_by_topic('/tf'))
        require(tf_publishers == ['robot_state_publisher', 'wheel_odometry'], f'Unexpected TF authorities: {tf_publishers}')
        subscriptions = probe.get_subscriber_names_and_types_by_node('wheel_odometry', '/')
        require({n for n,_ in subscriptions} <= {'/clock','/joint_states','/parameter_events'}, f'Odometry has unexpected inputs: {subscriptions}')
        topics = dict(probe.get_topic_names_and_types())
        require(not any('pose' in n or 'native' in n for n in topics), f'Ground-truth/native pose leaked into ROS: {topics}')
        result.update(status='PASS', initial=initial, final=final, odometry_messages=probe.count,
                      odometry_rate_hz=rate, tf_edges={k:sorted(v) for k,v in probe.edges.items()},
                      tf_publishers=tf_publishers, odometry_subscriptions=subscriptions, ros_topics=topics)
        (args.evidence/'frames.yaml').write_text(probe.buffer.all_frames_as_yaml())
    except Exception as exc:
        result['error'] = str(exc)
        raise
    finally:
        if probe:
            for _ in range(3):
                probe.command()
                rclpy.spin_once(probe, timeout_sec=0.05)
            probe.destroy_node()
        rclpy.shutdown()
        if process:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        if log:
            log.close()
        save_json(args.evidence/'result.json', result)
        print(json.dumps(result), flush=True)


if __name__ == '__main__':
    main()
