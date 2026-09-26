"""Offline guards against oracle leakage and misleading map-quality acceptance."""
import ast
from pathlib import Path
import unittest
import numpy as np
import yaml
from map_quality import quality, decode_pixels

ROOT = Path(__file__).resolve().parents[1]


class Contracts(unittest.TestCase):
    def test_unknown_grey_is_not_free_on_reload(self):
        decoded = decode_pixels(np.array([[0, 205, 254]]),
                                {'mode': 'trinary', 'negate': 0, 'occupied_thresh': .65, 'free_thresh': .196})
        self.assertEqual(decoded.tolist(), [[100, -1, 0]])

    def test_motion_and_initializer_have_no_pose_or_simulator_inputs(self):
        for script, permitted in [('mapping_survey.py', {'/cmd_vel', '/clock', '/firmware/state'}),
                                   ('initialize_localization.py', {'/initialpose', '/clock'})]:
            tree = ast.parse((ROOT/'scripts'/script).read_text())
            imports = [n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)]
            imports += [a.name for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names]
            self.assertFalse(any(x and x.startswith(('gz', 'tf2', 'nav_msgs', 'subprocess')) for x in imports))
            interfaces = {a.value for n in ast.walk(tree) if isinstance(n, ast.Call)
                          and isinstance(n.func, ast.Attribute)
                          and n.func.attr in {'create_subscription', 'create_publisher'}
                          for a in n.args if isinstance(a, ast.Constant) and isinstance(a.value, str)}
            self.assertEqual(interfaces, permitted)

    def test_modes_use_measurements_and_do_not_auto_seed_amcl(self):
        mapping = yaml.safe_load((ROOT/'simulation/config/mapping.yaml').read_text())['slam_toolbox']['ros__parameters']
        amcl = yaml.safe_load((ROOT/'simulation/config/localization.yaml').read_text())['amcl']['ros__parameters']
        self.assertEqual(mapping['scan_topic'], '/scan')
        self.assertEqual(mapping['base_frame'], 'base_link')
        self.assertEqual(mapping['odom_frame'], 'odom')
        self.assertGreater(mapping['transform_publish_period'], 0)
        self.assertEqual(amcl['scan_topic'], '/scan')
        self.assertEqual(amcl['base_frame_id'], 'base_drive')
        self.assertFalse(amcl['set_initial_pose'])
        self.assertTrue(amcl['always_reset_initial_pose'])
        self.assertTrue(amcl['tf_broadcast'])

    def test_quality_rejects_empty_and_unknown_maps(self):
        for grid in (np.zeros((130, 170)), np.full((130, 170), -1)):
            with self.assertRaises(AssertionError):
                quality(grid, .05, (-1.75, -1.75))

    def test_quality_rejects_displaced_walls_without_fitting_alignment(self):
        yy, xx = np.indices((130, 170))
        x, y = (xx+.5)*.05-4.25, (yy+.5)*.05-3.25
        wall = ((abs(abs(x)-3.94)<.04) & (abs(y)<2.96)) | ((abs(abs(y)-2.94)<.04) & (abs(x)<3.96))
        grid = np.where(wall, 100, 0)
        result = quality(grid, .05, (-1.75, -1.75))
        self.assertGreater(result['interior_known_fraction'], .99)
        with self.assertRaises(AssertionError):
            quality(grid, .05, (-1.35, -1.75))


if __name__ == '__main__':
    unittest.main()
