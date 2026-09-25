"""M4 live contract, physics observations and recording. No estimator is added."""
import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import signal
import statistics
import subprocess
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, Image, CameraInfo, Imu, JointState
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from rover_interfaces.msg import WheelEncoders
from tf2_ros import Buffer, TransformListener

ROOT = Path(__file__).resolve().parents[1]
SPECS = {'/scan': (LaserScan, 'lidar_link', 10),
         '/camera/image_raw': (Image, 'camera_optical_frame', 10),
         '/camera/camera_info': (CameraInfo, 'camera_optical_frame', 10),
         '/imu/data_raw': (Imu, 'imu_link', 100),
         '/wheel/encoders': (WheelEncoders, 'base_link', 50)}
BAG_TOPICS = [*SPECS, '/clock', '/tf', '/tf_static', '/joint_states', '/odom', '/robot_description']


def stamp(m):
    return m.header.stamp.sec*10**9+m.header.stamp.nanosec


def require(ok, message):
    if not ok:
        raise AssertionError(message)


class Probe(Node):
    def __init__(self):
        super().__init__('m4_acceptance')
        self.clock = 0
        self.messages = defaultdict(list)
        self.joints = {}
        self.ages = defaultdict(list)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self.on_joint, qos_profile_sensor_data)
        for topic, (typ, _, _) in SPECS.items():
            self.create_subscription(typ, topic, lambda m, t=topic: self.on_sensor(t, m), qos_profile_sensor_data)

    def on_clock(self, m):
        self.clock = m.clock.sec*10**9+m.clock.nanosec

    def on_joint(self, m):
        self.joints[stamp(m)] = dict(zip(m.name, m.position))

    def on_sensor(self, topic, m):
        self.messages[topic].append(m)
        if self.clock:
            self.ages[topic].append((self.clock-stamp(m))*1e-9)

    def wait(self, predicate, timeout=120):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)
            if predicate():
                return
        raise TimeoutError(f'Observation deadline; counts: { {t:len(v) for t,v in self.messages.items()} }')

    def command(self, v=0., w=0.):
        m = Twist(); m.linear.x = v; m.angular.z = w; self.pub.publish(m)

    def drive(self, duration, v=0., w=0.):
        start = self.clock
        last = 0
        def tick():
            nonlocal last
            if time.monotonic()-last > 0.1:
                self.command(v, w); last = time.monotonic()
            return self.clock-start >= duration*1e9
        self.wait(tick, max(120, duration*30))

    def contract(self):
        report = {}
        for topic, (_, frame, rate) in SPECS.items():
            messages = self.messages[topic]
            times = [stamp(m) for m in messages]
            require(len(times) >= rate*2, f'{topic}: insufficient samples')
            require(all(m.header.frame_id == frame for m in messages), f'{topic}: wrong frame')
            require(times[0] > 0 and all(b>a for a,b in zip(times,times[1:])), f'{topic}: invalid timestamps')
            actual = (len(times)-1)*1e9/(times[-1]-times[0])
            require(rate*0.8 < actual < rate*1.1, f'{topic}: unexpected simulation rate {actual}')
            require(self.count_publishers(topic) == 1, f'{topic}: publisher authority')
            self.wait(lambda: self.buffer.can_transform('odom', frame, Time(nanoseconds=times[-1])), 15)
            ages = self.ages[topic][10:]
            require(min(ages) > -0.15 and max(ages) < 0.5, f'{topic}: stale/future stamps {min(ages), max(ages)}')
            report[topic] = {'messages':len(times), 'rate_hz_sim':actual, 'frame':frame,
                             'first_stamp_ns':times[0], 'last_stamp_ns':times[-1],
                             'observed_age_s_min_max':[min(ages),max(ages)]}
        topics = dict(self.get_topic_names_and_types())
        require(not any('native' in t or 'pose' in t or t == '/map' for t in topics), 'Privileged pose input exposed')
        require(self.count_publishers('/odom') == 1, 'Odometry authority changed')
        subs = self.get_subscriber_names_and_types_by_node('wheel_odometry', '/')
        require({t for t,_ in subs} <= {'/clock','/joint_states','/parameter_events'}, 'Wheel odometry inputs changed')
        expected = {'base_link','left_wheel','right_wheel','rear_support','parcel_tray',
                    'lidar_link','camera_link','camera_optical_frame','imu_link'}
        import yaml
        frames = yaml.safe_load(self.buffer.all_frames_as_yaml())
        require(set(frames) == expected, f'Unexpected TF tree {frames}')
        require(frames['camera_optical_frame']['parent'] == 'camera_link', 'Optical frame parent')
        optical = self.buffer.lookup_transform('camera_link','camera_optical_frame',Time()).transform.rotation
        # REP-103 rotation (-pi/2, 0, -pi/2), up to quaternion sign.
        require(abs(abs(-.5*optical.x+.5*optical.y-.5*optical.z+.5*optical.w)-1)<1e-6, 'Optical axes wrong')
        return report

    def stationary(self):
        scans = self.messages['/scan'][-30:]
        for m in scans:
            require(len(m.ranges)==361 and abs(m.angle_min+math.pi)<1e-5 and abs(m.angle_max-math.pi)<1e-5, 'Scan geometry')
            require(abs(m.range_min-.12)<1e-5 and abs(m.range_max-8)<1e-5, 'Scan limits')
            require(all((math.isfinite(x) and m.range_min<=x<=m.range_max) or x==math.inf for x in m.ranges), 'Invalid range')
            # Test-only room geometry. Lidar at (-2.30,-1.50,0.58).
            require(abs(m.ranges[180]-6.24)<.06, 'East wall distance / lidar frame incorrect')
            require(abs(m.ranges[270]-4.44)<.06, 'North wall distance incorrect')
            require(abs(m.ranges[185]-6.24/math.cos(math.radians(5)))<.07, 'Low crate must be below scan plane')
            require(abs(m.ranges[225]-1.75*math.sqrt(2))<.07, 'Table must occlude wall at 45 degrees')
        sigma = statistics.stdev(m.ranges[180] for m in scans)
        require(.002 < sigma < .03, f'LiDAR noise absent/excessive {sigma}')
        images = self.messages['/camera/image_raw'][-30:]
        infos = {stamp(m):m for m in self.messages['/camera/camera_info']}
        matched = 0
        for m in images:
            require(m.encoding=='rgb8' and (m.width,m.height,m.step)==(320,240,960) and len(m.data)==230400, 'RGB contract')
            require(max(m.data)-min(m.data)>40, 'Blank camera image')
            if stamp(m) in infos:
                c=infos[stamp(m)]; matched+=1
                require((c.width,c.height)==(m.width,m.height), 'CameraInfo dimensions')
                require(abs(c.k[0]-160/math.tan(math.pi/6))<.1 and abs(c.k[0]-c.k[4])<.01 and c.k[8]==1, f'Camera calibration: {list(c.k)}')
        require(matched>=len(images)*.8, 'CameraInfo not synchronized')
        imus = self.messages['/imu/data_raw'][-200:]
        for m in imus:
            require(m.orientation_covariance[0]==-1 and m.orientation.w==0, 'Perfect orientation leaked')
            require(all(math.isfinite(getattr(v,k)) for v in [m.angular_velocity,m.linear_acceleration] for k in 'xyz'), 'Nonfinite IMU')
            require(abs(m.angular_velocity_covariance[0]-4e-6)<1e-12 and abs(m.linear_acceleration_covariance[0]-.0004)<1e-12, 'IMU covariance')
        az = statistics.mean(m.linear_acceleration.z for m in imus)
        gz = statistics.mean(m.angular_velocity.z for m in imus)
        gyro_sigma=statistics.stdev(m.angular_velocity.z for m in imus)
        require(abs(az-9.84)<.03 and abs(gz-.003)<.001, f'IMU gravity/bias {az,gz}')
        require(.001<gyro_sigma<.004, 'IMU noise')
        return {'lidar_east_mean_m':statistics.mean(m.ranges[180] for m in scans),
                'lidar_noise_stddev_m':sigma, 'imu_accel_z_mean':az, 'imu_gyro_z_mean':gz,
                'imu_gyro_z_stddev':gyro_sigma, 'synchronized_camera_samples':matched}

    def encoders(self):
        samples = self.messages['/wheel/encoders']
        origins = []
        for a,b in zip(samples,samples[1:]):
            require(b.sample_index>a.sample_index, 'Encoder sequence not increasing')
            if b.sample_index==a.sample_index+1:
                require(b.sample_period.sec*10**9+b.sample_period.nanosec==stamp(b)-stamp(a), 'Sample period mismatch')
        for m in samples:
            require(m.counts_per_revolution==2048 and m.left_joint_name=='left_wheel_joint' and m.right_joint_name=='right_wheel_joint','Encoder metadata')
            joints=self.joints.get(stamp(m))
            if joints:
                origins.append([joints[m.left_joint_name]-m.left_count*2*math.pi/2048,
                                joints[m.right_joint_name]-m.right_count*2*math.pi/2048])
        require(len(origins)>50, 'No matched joint/count stamps')
        for side in (0,1):
            values=[o[side] for o in origins]
            require(max(values)-min(values)<=2*math.pi/2048+1e-8, 'Encoder quantization disagrees with measured joint rotation')
        return {'matched_samples':len(origins), 'resolution_counts_per_rev':2048,
                'max_angle_error_rad':math.pi/2048}


def stop_process(p):
    if p and p.poll() is None:
        p.send_signal(signal.SIGINT)
        try: p.wait(timeout=25)
        except subprocess.TimeoutExpired:
            p.terminate(); p.wait(timeout=10)


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--evidence',type=Path,required=True)
    parser.add_argument('--existing',action='store_true'); parser.add_argument('--stationary-only',action='store_true')
    args=parser.parse_args(); args.evidence.mkdir(parents=True,exist_ok=True)
    result={'status':'FAIL'}; launch=recorder=probe=None; logs=[]
    rclpy.init()
    try:
        if not args.existing:
            log=(args.evidence/'launch.log').open('w'); logs.append(log)
            launch=subprocess.Popen(['python3',str(ROOT/'scripts/rover_sim.py'),'--drive','--sensors','--evidence',str(args.evidence/'runtime')],stdout=log,stderr=subprocess.STDOUT)
        probe=Probe()
        probe.wait(lambda: all(len(probe.messages[t])>=10 for t in SPECS))
        probe.drive(3.)
        result['stationary']=probe.stationary()
        if not args.stationary_only:
            log=(args.evidence/'record.log').open('w'); logs.append(log)
            recorder=subprocess.Popen(['ros2','bag','record','--use-sim-time','-s','mcap','-o',str(args.evidence/'sensors-bag'),'--topics',*BAG_TOPICS],stdout=log,stderr=subprocess.STDOUT)
            probe.drive(1.)
            result['motion']={}
            for name,v,w in [('forward',.2,0.),('reverse',-.2,0.),('left',0.,.4)]:
                before=probe.messages['/wheel/encoders'][-1]
                imu_start=len(probe.messages['/imu/data_raw'])
                probe.drive(1.2,v,w)
                moving_imus=probe.messages['/imu/data_raw'][imu_start+20:]
                if w:
                    require(statistics.mean(m.angular_velocity.z for m in moving_imus)>.25,
                            'IMU gyro did not observe the physical turn')
                probe.drive(.3)
                after=probe.messages['/wheel/encoders'][-1]
                dl,dr=after.left_count-before.left_count,after.right_count-before.right_count
                require((dl*v>0 and dr*v>0) if v else (dl<0<dr), f'{name}: encoder direction wrong')
                require(min(abs(dl),abs(dr))>100, f'{name}: encoder motion too small')
                result['motion'][name]={'left_counts':dl,'right_counts':dr}
            probe.drive(1.)
            stop_process(recorder)
            require(recorder.returncode==0, 'rosbag recording failed')
        result['interfaces']=probe.contract()
        result['encoders']=probe.encoders()
        (args.evidence/'frames.yaml').write_text(probe.buffer.all_frames_as_yaml())
        result['status']='PASS'
    except Exception as exc:
        result['error']=str(exc); raise
    finally:
        if probe:
            for _ in range(3):
                probe.command(); rclpy.spin_once(probe,timeout_sec=.02)
            probe.destroy_node()
        rclpy.shutdown(); stop_process(recorder); stop_process(launch)
        for f in logs: f.close()
        (args.evidence/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result),flush=True)


if __name__=='__main__': main()
