"""Planar differential-drive integration; no ROS or simulator dependencies."""
import math


class WheelOdometry:
    def __init__(self, radius, separation, axle_x):
        if radius <= 0 or separation <= 0:
            raise ValueError("Wheel radius and separation must be positive")
        self.radius, self.separation, self.axle_x = radius, separation, axle_x
        self.x, self.y, self.yaw = axle_x, 0.0, 0.0  # axle position in odom
        self.previous = None

    def update(self, stamp, left, right):
        if not all(math.isfinite(v) for v in (stamp, left, right)):
            return None
        if self.previous is None:
            self.previous = stamp, left, right
            return None
        t0, l0, r0 = self.previous
        if stamp <= t0:  # Reject duplicate/backwards stamps; restart for a world reset.
            return None
        dl, dr = self.radius * (left - l0), self.radius * (right - r0)
        ds, da = (dl + dr) / 2, (dr - dl) / self.separation
        # Exact constant-curvature arc (sinc avoids a small-angle singularity).
        half = da / 2
        distance = ds * (math.sin(half) / half if abs(half) > 1e-9 else 1.0)
        self.x += distance * math.cos(self.yaw + half)
        self.y += distance * math.sin(self.yaw + half)
        self.yaw += da
        self.previous = stamp, left, right
        dt = stamp - t0
        # base_link lies axle_x behind the wheel axle. Its lateral velocity is -a*w.
        return (self.x - self.axle_x * math.cos(self.yaw),
                self.y - self.axle_x * math.sin(self.yaw), self.yaw,
                ds / dt, -self.axle_x * da / dt, da / dt)
