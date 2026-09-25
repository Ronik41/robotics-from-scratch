import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/rover_sensors'))
from models import encoder_count


class Encoders(unittest.TestCase):
    def test_signed_revolutions_and_origin(self):
        for revolutions in [-3, -1, 0, 1, 3]:
            self.assertEqual(encoder_count(0.7+revolutions*2*math.pi, 0.7, 2048), revolutions*2048)

    def test_half_count_error_bound(self):
        for i in range(-1000, 1000):
            angle = i/71
            measured = encoder_count(angle, 0., 2048)*2*math.pi/2048
            self.assertLessEqual(abs(angle-measured), math.pi/2048+1e-12)

    def test_invalid_measurement(self):
        for value in [float('nan'), float('inf')]:
            with self.assertRaises(ValueError):
                encoder_count(value, 0, 2048)
