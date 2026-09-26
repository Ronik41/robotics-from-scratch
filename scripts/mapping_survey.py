"""Bounded teleoperation tapes. No map, pose, odometry, or Gazebo inputs.

These are repeatable data-collection motions in the documented fresh room,
not a navigator or obstacle-avoidance system. All commands traverse M5.
"""
import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

# (name, simulation seconds, body forward m/s, yaw rad/s)
TAPES = {
    'mapping': [
        ('settle', 1.0, 0., 0.),
        ('south_out', 12., .22, 0.), ('brake', .7, 0., 0.),
        ('south_back', 12., -.22, 0.), ('brake', .7, 0., 0.),
        ('west_clearance', 2., -.22, 0.), ('brake', .7, 0., 0.),
        ('face_north', 4.1, 0., .5), ('brake', .7, 0., 0.),
        ('west_out', 15., .22, 0.), ('brake', .7, 0., 0.),
        ('west_back', 15., -.22, 0.), ('brake', .7, 0., 0.),
        ('face_east', 4.1, 0., -.5), ('brake', .7, 0., 0.),
        ('return_near_start', 2., .22, 0.), ('settle', 2., 0., 0.)],
    'localization': [
        ('observe', 1., 0., 0.), ('forward', 2., .15, 0.),
        ('brake', .7, 0., 0.), ('reverse', 2., -.15, 0.),
        ('brake', .7, 0., 0.), ('left', 3.5, 0., .4),
        ('brake', .7, 0., 0.), ('right', 3.5, 0., -.4),
        ('settle', 2., 0., 0.)],
}


class Survey(Node):
    def __init__(self):
        super().__init__('m6_survey')
        self.clock = None
        self.state = None
        self.last_stamp = -1
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 1)
        self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        self.create_subscription(String, '/firmware/state', self.on_state, 10)
        self.clock_received = self.state_received = time.monotonic()

    def on_clock(self, msg):
        stamp = msg.clock.sec * 10**9 + msg.clock.nanosec
        if self.clock is not None and stamp < self.clock:
            raise RuntimeError('Clock reset: restart the complete scenario')
        if stamp != self.clock:
            self.clock_received = time.monotonic()
        self.clock = stamp

    def on_state(self, msg):
        self.state = json.loads(msg.data)
        self.state_received = time.monotonic()

    def command(self, v=0., w=0.):
        if self.clock is None or self.clock <= self.last_stamp:
            return
        self.last_stamp = self.clock
        msg = TwistStamped()
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(self.clock, 10**9)
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x, msg.twist.angular.z = v, w
        self.pub.publish(msg)

    def run(self, profile):
        end = time.monotonic() + 90
        while self.clock is None or self.state is None or not self.pub.get_subscription_count():
            if time.monotonic() > end:
                raise TimeoutError('Clock/firmware command interface not ready')
            rclpy.spin_once(self, timeout_sec=.02)
        # Endpoint discovery can precede node-name discovery. Wait while names
        # are unresolved, before sending any command, rather than accepting an
        # anonymous subscriber or mistaking a recorder for an actuator bypass.
        while True:
            consumers = [i.node_name for i in self.get_subscriptions_info_by_topic('/cmd_vel')]
            if consumers and all('UNKNOWN' not in name for name in consumers) and 'sim_firmware' in consumers:
                break
            if time.monotonic() > end:
                raise TimeoutError(f'Command consumer discovery incomplete: {consumers}')
            rclpy.spin_once(self, timeout_sec=.05)
        # A rosbag recorder is an observer, not another actuator. Reject bypasses.
        if consumers.count('sim_firmware') != 1 or set(consumers) - {'sim_firmware', 'rosbag2_recorder'}:
            raise RuntimeError(f'Unexpected command consumers: {consumers}')
        trace = []
        for name, duration, v, w in TAPES[profile]:
            print(f'{name}: {duration}s, v={v}, w={w}', flush=True)
            start = self.clock
            end = time.monotonic() + max(60, duration * 15)
            last_send = 0
            while (self.clock - start) * 1e-9 < duration:
                now = time.monotonic()
                if now > end or now - self.clock_received > 2 or now - self.state_received > 2:
                    raise TimeoutError('Stalled clock or firmware telemetry; command stream stopped')
                allowed = {'ACTIVE', 'IDLE', 'WAIT_COMMAND', 'COMMAND_TIMEOUT'}
                if v == 0. and w == 0.:
                    # Renderer startup may have expired a lease before the first command.
                    # Fresh zero intent can re-establish IDLE; no latch is ever cleared.
                    allowed.add('CLOCK_STALLED')
                if self.state['state'] not in allowed:
                    raise RuntimeError(f"Firmware inhibited survey: {self.state['state']}")
                if now - last_send >= .05:
                    self.command(v, w)
                    last_send = now
                rclpy.spin_once(self, timeout_sec=.01)
            trace.append({'phase': name, 'start_sim_s': start*1e-9,
                          'end_sim_s': self.clock*1e-9, 'v': v, 'w': w,
                          'firmware': self.state})
        return trace


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', choices=TAPES, default='mapping')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    rclpy.init(); node = Survey()
    try:
        trace = node.run(args.profile)
        if args.output:
            args.output.write_text(json.dumps({'profile': args.profile, 'phases': trace}, indent=2)+'\n')
    finally:
        node.command()
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
