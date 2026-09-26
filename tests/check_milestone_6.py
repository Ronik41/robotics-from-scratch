"""Real M6 mapping/localization acceptance. Gazebo pose is ONLY a test oracle.

The motion tape and operator initializer are separate processes with no oracle
inputs. Results include message-level TF authority, ROS input graph, map geometry,
timestamp-matched trajectory errors, covariance, safety state and an MCAP bag.
"""
import argparse
from collections import defaultdict, deque
import hashlib
import json
import math
from pathlib import Path
import signal
import subprocess
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V
import yaml
from PIL import Image

from map_quality import quality, decode_pixels

ROOT = Path(__file__).resolve().parents[1]
BAG_TOPICS = ['/clock', '/scan', '/odom', '/tf', '/tf_static', '/map', '/amcl_pose',
              '/initialpose', '/cmd_vel', '/wheel/encoders', '/firmware/state',
              '/motor/driver_state', '/robot_description']


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def angle(x):
    return math.atan2(math.sin(x), math.cos(x))


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9


def stop(process):
    if process and process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=25)
        except subprocess.TimeoutExpired:
            process.terminate(); process.wait(timeout=10)


class Probe(Node):
    def __init__(self):
        super().__init__('m6_acceptance_oracle')
        self.clock = 0.
        self.map = self.odom = self.pose = self.state = self.driver = None
        self.map_count = self.pose_count = 0
        self.authorities = defaultdict(set)
        self.edge_counts = defaultdict(int)
        self.static_edges = set()
        self.truth = deque(maxlen=10000)
        self.truth_lock = threading.Lock()
        self.samples, self.states = [], []
        self.last_sample = -1.
        self.buffer = Buffer(); self.listener = TransformListener(self.buffer, self)
        self.create_subscription(Clock, '/clock', lambda m: setattr(self, 'clock', m.clock.sec+m.clock.nanosec*1e-9), qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'odom', m), 100)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, 100)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 100)
        self.create_subscription(TFMessage, '/tf_static', self.on_static,
                                 QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, '/firmware/state', self.on_state, 100)
        self.create_subscription(String, '/motor/driver_state', lambda m: setattr(self, 'driver', json.loads(m.data)), 100)
        self.gz = GzNode()
        self.gz.subscribe(Pose_V, '/world/delivery_room/pose/info', self.on_truth)

    def on_map(self, msg):
        self.map = msg; self.map_count += 1

    def on_pose(self, msg):
        self.pose = msg; self.pose_count += 1

    def on_state(self, msg):
        self.state = json.loads(msg.data); self.states.append(self.state)

    def on_tf(self, msg):
        for tf in msg.transforms:
            edge = (tf.header.frame_id, tf.child_frame_id)
            self.authorities[edge]  # Record the edge; the C++ probe resolves its publisher GID.
            self.edge_counts[edge] += 1
            t, q = tf.transform.translation, tf.transform.rotation
            require(all(math.isfinite(v) for v in (t.x, t.y, t.z, q.x, q.y, q.z, q.w)), 'Nonfinite TF')
            require(abs(sum(v*v for v in (q.x, q.y, q.z, q.w))-1) < 1e-5, 'Invalid TF quaternion')

    def on_static(self, msg):
        self.on_tf(msg)
        self.static_edges.update((t.header.frame_id, t.child_frame_id) for t in msg.transforms)

    def on_truth(self, msg):
        for pose in msg.pose:
            if pose.name == 'delivery_rover':
                with self.truth_lock:
                    self.truth.append((msg.header.stamp.sec+msg.header.stamp.nsec*1e-9,
                                       pose.position.x, pose.position.y, yaw(pose.orientation)))

    def sample(self):
        if self.odom is None or stamp(self.odom)-self.last_sample < .19:
            return
        # Use a slightly delayed TF time so both transport paths have delivered.
        t = stamp(self.odom)-.1
        if t <= 0 or not self.buffer.can_transform('map', 'base_link', Time(seconds=t)):
            return
        with self.truth_lock:
            if not self.truth:
                return
            truth = min(self.truth, key=lambda p: abs(p[0]-t))
        if abs(truth[0]-t) > .015:
            return
        tf = self.buffer.lookup_transform('map', 'base_link', Time(seconds=t)).transform
        od = self.buffer.lookup_transform('odom', 'base_link', Time(seconds=t)).transform
        estimate = [tf.translation.x, tf.translation.y, yaw(tf.rotation)]
        odom = [od.translation.x, od.translation.y, yaw(od.rotation)]
        expected = [truth[1]+2.5, truth[2]+1.5, truth[3]]
        self.samples.append({'time': t, 'truth_time': truth[0], 'truth_in_map': expected,
                             'estimate': estimate, 'odom': odom,
                             'position_error_m': math.dist(estimate[:2], expected[:2]),
                             'yaw_error_rad': abs(angle(estimate[2]-expected[2])),
                             'odom_position_error_m': math.dist(odom[:2], expected[:2]),
                             'odom_yaw_error_rad': abs(angle(odom[2]-expected[2]))})
        self.last_sample = stamp(self.odom)

    def wait(self, predicate, timeout=120):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.01)
            self.sample()
            if predicate():
                return
        raise TimeoutError('M6 observation deadline')

    def coast(self, seconds):
        start = self.clock
        self.wait(lambda: self.clock-start >= seconds, max(60, seconds*15))

    def audit(self, mode, runtime, authority_file):
        owner = 'slam_toolbox' if mode == 'mapping' else 'amcl'
        endpoints = {bytes(i.endpoint_gid).hex(): i.node_name for topic in ('/tf', '/tf_static')
                     for i in self.get_publishers_info_by_topic(topic)}
        observed = defaultdict(set)
        for line in authority_file.read_text().splitlines():
            parent, child, kind, gid, count = line.split('\t')
            require(int(count) > 0, 'TF authority probe received no messages')
            observed[(parent, child)].add(gid)
        require(set(observed) == set(self.authorities), 'C++ and Python TF edge observations differ')
        self.authorities = observed
        authorities = {' -> '.join(edge): sorted(endpoints.get(g, 'UNKNOWN:'+g) for g in gids)
                       for edge, gids in self.authorities.items()}
        require(authorities.get('map -> odom') == [owner], f'map TF authority: {authorities}')
        require(self.edge_counts[('map', 'odom')] > 20, 'map -> odom is not live')
        require(('map', 'odom') not in self.static_edges, 'map -> odom must not be static')
        require(authorities.get('odom -> base_link') == ['wheel_odometry'], 'Wheel odometry authority changed')
        require(all(len(gids) == 1 for gids in self.authorities.values()), 'Duplicate TF authority')
        parents = defaultdict(set)
        for parent, child in self.authorities:
            parents[child].add(parent)
        require(all(len(p) == 1 for p in parents.values()), 'Conflicting TF parents')
        maps = self.get_publishers_info_by_topic('/map')
        require([i.node_name for i in maps] == [owner if mode == 'mapping' else 'map_server'], 'Map publisher ownership')
        graph = {}
        allowed = {
            'slam_toolbox': {'/scan', '/tf', '/tf_static', '/clock', '/parameter_events', '/bond', '/slam_toolbox/feedback'},
            'amcl': {'/scan', '/tf', '/tf_static', '/clock', '/parameter_events', '/map', '/initialpose', '/bond'},
            'wheel_odometry': {'/joint_states', '/clock'},
            'sim_firmware': {'/wheel/encoders', '/clock', '/cmd_vel'},
            'm6_survey': {'/clock', '/firmware/state'},
        }
        nodes = {n for n, ns in self.get_node_names_and_namespaces() if ns == '/'}
        require(('amcl' not in nodes) if mode == 'mapping' else ('slam_toolbox' not in nodes), 'Competing estimators')
        require(not nodes.intersection({'planner_server', 'controller_server', 'bt_navigator', 'waypoint_follower'}), 'M7 node present')
        for name in (owner, 'wheel_odometry', 'sim_firmware'):
            topics = self.get_subscriber_names_and_types_by_node(name, '/')
            graph[name] = topics
            require({t for t, _ in topics} <= allowed[name], f'Unexpected estimator/control input: {name}: {topics}')
            require(topics, f'Missing node: {name}')
        # SLAM Toolbox creates its marker-feedback endpoint even when interactive
        # mode is disabled. It must have no input source in this workflow.
        require(not self.get_publishers_info_by_topic('/slam_toolbox/feedback'), 'External SLAM pose-graph edits are not part of acceptance')
        for name in nodes:
            if name.startswith('transform_listener_impl_'):
                topics = self.get_subscriber_names_and_types_by_node(name, '/')
                graph[name] = topics
                require({t for t, _ in topics} <= {'/tf', '/tf_static', '/clock', '/parameter_events'}, 'Unexpected internal TF-listener input')
        cmd = self.get_subscriptions_info_by_topic('/cmd_vel')
        consumers = [i.node_name for i in cmd]
        require(consumers.count('sim_firmware') == 1 and not set(consumers)-{'sim_firmware', 'rosbag2_recorder'}, 'Motion bypasses M5')
        model = ET.parse(runtime/'rover.sdf').getroot().find('model')
        plugins = [p.attrib['name'] for p in model.findall('plugin')]
        require(plugins.count('rover::MotorDriver') == 1 and not any('DiffDrive' in p for p in plugins), 'Actuation bypass')
        bridge = yaml.safe_load((ROOT/'simulation/config/bridge-firmware.yaml').read_text())
        native = {b['gz_topic_name'] for b in bridge}
        require(native == {'/world/delivery_room/clock', '/model/delivery_rover/joint_state',
                           '/sensors/lidar', '/sensors/camera/image', '/sensors/camera/camera_info',
                           '/motor/driver_state'}, 'Native bridge input allowlist changed')
        require(not any('pose/info' in t or t.startswith('/world/') for t, _ in self.get_topic_names_and_types()), 'Gazebo pose exposed to ROS')
        return {'tf_authorities': authorities, 'dynamic_map_tf_messages': self.edge_counts[('map', 'odom')],
                'subscriptions': graph, 'native_bridge_allowlist': sorted(native), 'model_plugins': plugins,
                'cmd_vel_consumers': [i.node_name for i in cmd], 'nodes': sorted(nodes)}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode', choices=['mapping', 'localization'], required=True)
    p.add_argument('--evidence', type=Path, required=True)
    p.add_argument('--map', type=Path, default=ROOT/'maps/delivery_room_v1/map.yaml')
    p.add_argument('--existing-runtime', type=Path, help='Run in an idle fresh desktop instead of creating a simulator')
    args = p.parse_args(); args.evidence.mkdir(parents=True, exist_ok=True)
    result = {'status': 'FAIL', 'mode': args.mode}; processes = []; logs = []
    recorder = probe = None
    runtime = args.existing_runtime or args.evidence/'runtime'

    def start(name, cmd):
        log = (args.evidence/(name+'.log')).open('w'); logs.append(log)
        proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT)
        processes.append(proc); return proc

    rclpy.init()
    try:
        with (args.evidence/'tf-probe-build.log').open('w') as build_log:
            subprocess.run(['cmake', '-S', str(ROOT/'tests/tf_authority'), '-B', '/tmp/m6-tf-probe'],
                           stdout=build_log, stderr=subprocess.STDOUT, check=True, timeout=90)
            subprocess.run(['cmake', '--build', '/tmp/m6-tf-probe', '-j2'],
                           stdout=build_log, stderr=subprocess.STDOUT, check=True, timeout=120)
        start('tf-probe', ['/tmp/m6-tf-probe/tf_authority_probe', str(args.evidence/'tf-authorities.tsv')])
        if not args.existing_runtime:
            start('launch', ['python3', str(ROOT/'scripts/rover_sim.py'), '--drive', '--sensors', '--firmware',
                             '--m6', args.mode, '--map', str(args.map), '--evidence', str(runtime)])
        probe = Probe()
        probe.wait(lambda: probe.map is not None and probe.odom is not None and probe.state is not None and probe.driver is not None)
        require(probe.map.header.frame_id == 'map', 'Map frame incorrect')
        require(probe.map.info.resolution > 0 and len(probe.map.data) == probe.map.info.width*probe.map.info.height, 'Invalid occupancy grid')
        require(all(-1 <= x <= 100 for x in probe.map.data), 'Invalid occupancy values')
        probe.coast(1.)
        if args.mode == 'localization':
            require(('map', 'odom') not in probe.authorities, 'AMCL fabricated a pose before initialization')
            result['no_map_tf_before_initial_pose'] = True
            saved_metadata = yaml.safe_load(args.map.read_text())
            saved_grid = decode_pixels(Image.open(args.map.with_suffix('.pgm')), saved_metadata)
            require(np.array_equal(saved_grid.ravel(), probe.map.data), 'Map server did not publish the saved grid faithfully')
            result['loaded_map_sha256'] = {f.name: hashlib.sha256(f.read_bytes()).hexdigest()
                                         for f in (args.map, args.map.with_suffix('.pgm'))}
        recorder = start('record', ['ros2', 'bag', 'record', '--use-sim-time', '-s', 'mcap',
                                    '--include-unpublished-topics',
                                    '--qos-profile-overrides-path', str(ROOT/'simulation/config/m6-record-qos.yaml'),
                                    '--storage-preset-profile', 'zstd_fast', '-o', str(args.evidence/'bag'),
                                    '--topics', *BAG_TOPICS])
        probe.coast(1.)
        if args.mode == 'localization':
            # Subscribe to the one-shot prior BEFORE its short-lived publisher
            # appears. Regular continuous streams do not reveal this race.
            probe.wait(lambda: any(i.node_name == 'rosbag2_recorder'
                                  for i in probe.get_subscriptions_info_by_topic('/initialpose')))
            init = start('initial-pose', ['python3', str(ROOT/'scripts/initialize_localization.py')])
            probe.wait(lambda: init.poll() is not None, 60)
            require(init.returncode == 0, 'Initial pose command failed')
            result['approximate_initial_pose'] = [.25, -.20, .20]
            result['initial_pose_body_frame'] = 'base_drive'
            result['initial_sigma'] = [.25, .25, .30]
        probe.wait(lambda: probe.edge_counts[('map', 'odom')] > 20)
        # Fail an interface/ownership error before running the longer motion tape.
        result['preflight_graph'] = probe.audit(args.mode, runtime, args.evidence/'tf-authorities.tsv')
        survey = start('survey', ['python3', str(ROOT/'scripts/mapping_survey.py'), '--profile', args.mode,
                                  '--output', str(args.evidence/'survey.json')])
        probe.wait(lambda: survey.poll() is not None, 1500)
        require(survey.returncode == 0, 'Survey failed; see survey.log')
        probe.coast(3.)
        require(probe.state['state'] == 'COMMAND_TIMEOUT', 'M5 command lease did not expire')
        require(max(abs(x) for x in probe.driver['wheel_rad_s']) < .1, 'Rover did not stop')
        require(probe.state['rejected_commands'] == 0, 'Nominal survey generated invalid commands')
        result['safety'] = {'firmware': probe.state, 'driver': probe.driver,
                            'max_effort_nm': max(max(abs(x) for x in s['effort_nm']) for s in probe.states)}
        require(result['safety']['max_effort_nm'] <= 2, 'Torque bound violated')
        result['graph'] = probe.audit(args.mode, runtime, args.evidence/'tf-authorities.tsv')
        m = probe.map
        grid = np.array(m.data).reshape((m.info.height, m.info.width))
        result['map_quality'] = quality(grid, m.info.resolution, (m.info.origin.position.x, m.info.origin.position.y))
        require(len(probe.samples) > 30, 'Too few timestamp-matched oracle comparisons')
        final = probe.samples[-10:]
        result['timestamp_matched_samples'] = len(probe.samples)
        result['max_oracle_time_mismatch_s'] = max(abs(s['time']-s['truth_time']) for s in probe.samples)
        result['trajectory'] = {key: {'rmse': float(np.sqrt(np.mean([s[key]**2 for s in probe.samples]))),
                                      'max': max(s[key] for s in probe.samples),
                                      'final_window_max': max(s[key] for s in final)}
                                for key in ('position_error_m', 'yaw_error_rad', 'odom_position_error_m', 'odom_yaw_error_rad')}
        require(result['trajectory']['position_error_m']['final_window_max'] < .15, 'Final localization position error exceeds 15 cm')
        require(result['trajectory']['yaw_error_rad']['final_window_max'] < .12, 'Final localization yaw error exceeds .12 rad')
        if args.mode == 'mapping':
            require(probe.map_count > 5, 'Map did not update during mapping')
            saver = start('save-map', ['bash', str(ROOT/'scripts/save-map.sh'), str(args.map.with_suffix(''))])
            probe.wait(lambda: saver.poll() is not None, 60)
            require(saver.returncode == 0, 'Map saver failed')
            image = Image.open(args.map.with_suffix('.pgm'))
            image.resize((image.width*4, image.height*4), Image.Resampling.NEAREST).save(args.map.with_suffix('.png'))
            metadata = yaml.safe_load(args.map.read_text())
            saved = decode_pixels(image, metadata)
            require(int(np.sum(saved < 0)) == int(np.sum(grid < 0)), 'Save/load changed unknown cells to free')
            require(metadata['origin'][2] == 0., 'Unexpected rotated map image')
            result['saved_map_quality'] = quality(saved, metadata['resolution'], metadata['origin'][:2])
        else:
            require(probe.pose_count > 10, 'AMCL did not repeatedly update pose')
            require(probe.pose.header.frame_id == 'map', 'AMCL pose frame')
            variances = [probe.pose.pose.covariance[i] for i in (0, 7, 35)]
            require(all(math.isfinite(v) and 0 < v < limit for v, limit in zip(variances, (.04, .04, .0225))), 'AMCL uncertainty did not converge')
            result['final_covariance_xy_yaw'] = variances
            result['amcl_pose_updates'] = probe.pose_count
            settled = [i for i in range(len(probe.samples)-9)
                       if all(s['position_error_m'] < .15 and s['yaw_error_rad'] < .12
                              for s in probe.samples[i:i+10])]
            require(settled, 'No sustained localization convergence window')
            result['first_converged_window_sim_s'] = [probe.samples[settled[0]]['time'],
                                                       probe.samples[settled[0]+9]['time']]
            # A fixed saved map must not be silently modified by localization.
            require(result['loaded_map_sha256'] == {f.name: hashlib.sha256(f.read_bytes()).hexdigest()
                    for f in (args.map, args.map.with_suffix('.pgm'))}, 'Saved map mutated')
        (args.evidence/'frames.yaml').write_text(probe.buffer.all_frames_as_yaml())
        stop(recorder); require(recorder.returncode == 0, 'Bag recorder failed')
        metadata = yaml.safe_load((args.evidence/'bag/metadata.yaml').read_text())['rosbag2_bagfile_information']
        counts = {x['topic_metadata']['name']: x['message_count'] for x in metadata['topics_with_message_count']}
        require(all(counts.get(t, 0) > 0 for t in ('/scan', '/odom', '/map', '/tf', '/tf_static', '/cmd_vel', '/firmware/state')), 'Incomplete bag')
        if args.mode == 'localization':
            require(counts.get('/amcl_pose', 0) > 10 and counts.get('/initialpose', 0) == 1, 'Localization bag missing pose evidence')
        result['bag_message_counts'] = counts
        result['status'] = 'PASS'
    except Exception as exc:
        result['error'] = str(exc)
        raise
    finally:
        for process in reversed(processes):
            stop(process)
        if probe:
            (args.evidence/'trajectory.json').write_text(json.dumps(probe.samples, indent=2)+'\n')
            probe.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        for log in logs: log.close()
        (args.evidence/'result.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result), flush=True)


if __name__ == '__main__':
    main()
