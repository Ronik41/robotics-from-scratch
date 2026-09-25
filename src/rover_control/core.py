"""Deterministic firmware model. Times are seconds; speeds rad/s; effort N m.

No ROS, Gazebo, pose, wall-clock calls or hidden threads in this module.
"""
import math
from dataclasses import dataclass


def clip(x, limit):
    return max(-limit, min(limit, x))


@dataclass(frozen=True)
class Limits:
    radius: float = .14
    track: float = .48
    max_speed: float = 4.
    acceleration: float = 4.
    torque: float = 2.
    deadband: float = .04
    kp: float = .45
    ki: float = .8
    kd: float = .002
    integral_limit: float = 1.2
    command_timeout: float = .5
    feedback_timeout: float = .2
    wall_timeout: float = 1.
    future_tolerance: float = .03


class Controller:
    def __init__(self, limits=Limits()):
        self.cfg = limits
        self.command_stamp = -math.inf
        self.command_wall = -math.inf
        self.feedback_stamp = -math.inf
        self.feedback_wall = -math.inf
        self.index = -1
        self.counts = None
        self.measured = [0., 0.]
        self.target = [0., 0.]
        self.ramped = [0., 0.]
        self.integral = [0., 0.]
        self.previous = [0., 0.]
        self.derivative = [0., 0.]
        self.effort = [0., 0.]
        self.estop = self.asserted = False
        self.zero_seen = False
        self.fault = 'WAIT_COMMAND'
        self.rejected_commands = self.rejected_encoders = 0
        self.saturated = False
        self.state = 'WAIT_COMMAND'

    def inhibit(self, reason):
        self.fault = reason
        self.target = [0., 0.]
        self.clear_pid()

    def clear_pid(self):
        self.ramped = [0., 0.]
        self.integral = [0., 0.]
        self.derivative = [0., 0.]
        self.previous = list(self.measured)
        self.effort = [0., 0.]

    def command(self, stamp, values, frame, now, wall):
        # Unsupported axes must be zero, not silently ignored. Rejections stop
        # immediately and never refresh either freshness timestamp.
        valid = (len(values) == 6 and all(math.isfinite(x) for x in [stamp, *values])
                 and frame == 'base_link' and stamp > 0 and stamp > self.command_stamp
                 and -self.cfg.future_tolerance <= now-stamp <= self.cfg.command_timeout
                 and all(values[i] == 0 for i in (1, 2, 3, 4)))
        if not valid:
            self.rejected_commands += 1
            self.zero_seen = False
            self.inhibit('INVALID_COMMAND')
            return False
        v, w = values[0], values[5]
        self.zero_seen = v == 0 and w == 0
        raw = [(v-w*self.cfg.track/2)/self.cfg.radius,
               (v+w*self.cfg.track/2)/self.cfg.radius]
        # Finite input can overflow during kinematics.
        if not all(math.isfinite(x) for x in raw):
            self.rejected_commands += 1
            self.inhibit('INVALID_COMMAND')
            return False
        self.command_stamp, self.command_wall = stamp, wall
        scale = max(1., max(abs(x) for x in raw)/self.cfg.max_speed)
        self.saturated = scale > 1
        self.target = [x/scale for x in raw]
        self.fault = None
        return True

    def set_estop(self, asserted):
        self.asserted = asserted
        if asserted:
            self.estop = True
            self.zero_seen = False
            self.inhibit('ESTOP')

    def can_reset(self, now, wall):
        return (not self.asserted and self.zero_seen
                and now-self.command_stamp <= self.cfg.command_timeout
                and wall-self.command_wall <= self.cfg.wall_timeout
                and self.feedback_fresh(now, wall)
                and max(abs(x) for x in self.measured) < .15)

    def reset(self, now, wall):
        if not self.can_reset(now, wall):
            return False
        self.estop = False
        self.zero_seen = False
        self.inhibit('WAIT_COMMAND')
        # Preserve replay watermark; only a newer command can arm motion.
        return True

    def feedback_fresh(self, now, wall):
        return (self.counts is not None and self.index > 0
                and -self.cfg.future_tolerance <= now-self.feedback_stamp <= self.cfg.feedback_timeout
                and wall-self.feedback_wall <= self.cfg.wall_timeout)

    def reason(self, now, wall):
        if self.estop:
            return 'ESTOP'
        if self.fault:
            return self.fault
        if now-self.command_stamp > self.cfg.command_timeout or wall-self.command_wall > self.cfg.wall_timeout:
            return 'COMMAND_TIMEOUT'
        if not self.feedback_fresh(now, wall):
            return 'ENCODER_TIMEOUT'
        return 'IDLE' if self.target == [0., 0.] else 'ACTIVE'

    def tick(self, now, wall):
        self.state = self.reason(now, wall)
        if self.state != 'ACTIVE':
            self.clear_pid()
        return self.state

    def encoder(self, stamp, index, counts, cpr, period, now, wall, metadata=True):
        dt = stamp-self.feedback_stamp
        valid = (metadata and math.isfinite(stamp) and stamp > 0
                 and -self.cfg.future_tolerance <= now-stamp <= self.cfg.feedback_timeout
                 and cpr == 2048 and index > self.index and dt > 0
                 and len(counts) == 2 and all(isinstance(x, int) for x in counts)
                 and (self.counts is None or (math.isfinite(period) and 0 < period <= dt+1e-6)))
        if not valid:
            self.rejected_encoders += 1
            self.inhibit('INVALID_ENCODER')
            return False
        if self.counts is not None:
            measured = [(b-a)*2*math.pi/cpr/dt for a,b in zip(self.counts, counts)]
            if max(abs(x) for x in measured) > 20:
                self.rejected_encoders += 1
                self.inhibit('INVALID_ENCODER')
                return False
            self.measured = measured
        first = self.counts is None
        self.counts, self.index = list(counts), index
        self.feedback_stamp, self.feedback_wall = stamp, wall
        if first or dt > self.cfg.feedback_timeout:
            self.clear_pid()
            return True
        if self.tick(now, wall) != 'ACTIVE':
            return True
        for i in range(2):
            self.ramped[i] += clip(self.target[i]-self.ramped[i], self.cfg.acceleration*dt)
            error = self.ramped[i]-self.measured[i]
            # Low-pass derivative on measurement avoids setpoint kick and
            # reduces differentiation of quantized encoder count steps.
            alpha = dt/(.05+dt)
            self.derivative[i] += alpha*((self.measured[i]-self.previous[i])/dt-self.derivative[i])
            trial = clip(self.integral[i]+self.cfg.ki*error*dt, self.cfg.integral_limit)
            u = self.cfg.kp*error + trial - self.cfg.kd*self.derivative[i]
            if abs(u) <= self.cfg.torque or u*error < 0:
                self.integral[i] = trial  # conditional-integration anti-windup
            u = self.cfg.kp*error+self.integral[i]-self.cfg.kd*self.derivative[i]
            self.effort[i] = clip(u, self.cfg.torque)
            self.previous[i] = self.measured[i]
        return True

    def snapshot(self, now, wall):
        self.tick(now, wall)
        return {'state': self.state, 'estop_latched': self.estop,
                'estop_asserted': self.asserted, 'target_rad_s': self.target,
                'ramped_rad_s': self.ramped, 'measured_rad_s': self.measured,
                'effort_nm': self.effort, 'integral_nm': self.integral,
                'speed_saturated': self.saturated,
                'torque_saturated': any(abs(x) >= self.cfg.torque for x in self.effort),
                'command_age_s': now-self.command_stamp if math.isfinite(self.command_stamp) else None,
                'encoder_age_s': now-self.feedback_stamp if math.isfinite(self.feedback_stamp) else None,
                'rejected_commands': self.rejected_commands, 'rejected_encoders': self.rejected_encoders}
