"""Analytic cases catch sign, curvature, offset, and feedback-time mistakes."""
import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/rover_odometry'))
from kinematics import WheelOdometry


class KinematicsTests(unittest.TestCase):
    def model(self):
        model = WheelOdometry(0.14, 0.48, 0.14)
        self.assertIsNone(model.update(1.0, 7.0, 7.0))  # arbitrary encoder startup angle
        return model

    def test_forward_reverse(self):
        model = self.model()
        state = model.update(3.0, 7 + 1/0.14, 7 + 1/0.14)
        self.assertAlmostEqual(state[0], 1.0)
        self.assertAlmostEqual(state[3], 0.5)
        state = model.update(5.0, 7.0, 7.0)
        self.assertAlmostEqual(state[0], 0.0)
        self.assertAlmostEqual(state[3], -0.5)

    def test_quarter_turn_about_offset_axle(self):
        model = self.model()
        angle = math.pi/2
        rotation = angle * 0.48 / (2 * 0.14)
        x, y, yaw, vx, vy, wz = model.update(2.0, 7-rotation, 7+rotation)
        self.assertAlmostEqual(x, 0.14)
        self.assertAlmostEqual(y, -0.14)
        self.assertAlmostEqual(yaw, angle)
        self.assertAlmostEqual(vx, 0.0)
        self.assertAlmostEqual(vy, -0.14*angle)
        self.assertAlmostEqual(wz, angle)

    def test_arc(self):
        model = self.model()
        # Axle follows radius 1 m through pi/2, starting at (0.14, 0).
        angle = math.pi/2
        state = model.update(2.0, 7+angle*(1-0.24)/0.14, 7+angle*(1+0.24)/0.14)
        self.assertAlmostEqual(state[0], 1.14)
        self.assertAlmostEqual(state[1], 0.86)

    def test_reject_bad_or_stale_samples_without_changing_reference(self):
        model = self.model()
        for sample in [(1, 100, 100), (0.5, 100, 100), (2, math.nan, 0)]:
            self.assertIsNone(model.update(*sample))
        self.assertAlmostEqual(model.update(2, 8, 8)[0], 0.14)

    def test_stationary(self):
        state = self.model().update(2, 7, 7)
        self.assertEqual(state, (0., 0., 0., 0., -0., 0.))


if __name__ == '__main__':
    unittest.main()
