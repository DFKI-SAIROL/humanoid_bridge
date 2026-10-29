import unittest
from types import SimpleNamespace as S
from unittest.mock import patch, Mock
import numpy as np
from h1_2_viewer.sensors import depth_points, SensorScene
from h1_2_viewer.spatial_data import FrameGraph, Pose
from h1_2_viewer.head_camera_ros import decode_depth


class Sensors(unittest.TestCase):
    def test_depth_rate_limits_projection_and_image_logging(self):
        from h1_2_viewer.renderer import Dashboard
        from h1_2_viewer.configuration import load_profile
        d = Dashboard.__new__(Dashboard)
        d.profile = load_profile()
        d._rates, d._images, d.last, d.errors = {}, {}, {}, {}
        d.counts = {'head_depth': 0}
        d.sensors = S(cloud=Mock())
        with patch('rerun.log') as log, patch('rerun.set_time'):
            for i in range(100):
                d.head_depth(np.full((8, 8), 1 + i*.001), np.eye(3), 'optical', elapsed=i/100)
        self.assertEqual(d.sensors.cloud.call_count, 5)
        self.assertEqual(sum(c.args[0] == 'cameras/head/depth' for c in log.call_args_list), 5)
        self.assertEqual(d.counts['head_depth'], 100)

    def test_projection_and_invalid_depth(self):
        xyz, _ = depth_points([[2, 0], [float('nan'), 4]], [[2, 0, 0], [0, 2, 0], [0, 0, 1]], stride=1)
        np.testing.assert_allclose(xyz, [[0, 0, 2], [2, 2, 4]])
        with self.assertRaises(ValueError):
            depth_points([[1]], [[0, 0, 0], [0, 2, 0], [0, 0, 1]])

    def test_padded_big_endian_depth(self):
        msg = S(encoding='16UC1', is_bigendian=True, width=1, height=2, step=4,
                data=b'\x03\xe8\x00\x00\x07\xd0\x00\x00')
        np.testing.assert_allclose(decode_depth(msg), [[1], [2]])

    def test_missing_tf_waits_then_resolves(self):
        graph = FrameGraph('map')
        spatial = S(graph=graph, robot_frame='base', resolve=lambda x: x)
        scene = SensorScene(spatial)
        with patch('h1_2_viewer.sensors.rr.log') as log:
            scene.cloud('lidar', [[1, 0, 0]], 'lidar', [0, 255, 255])
            self.assertTrue(any('Clear' in type(c.args[1]).__name__ for c in log.call_args_list))
            graph.set('lidar', Pose('base', [0, 0, 1]), static=True)
            scene.refresh()
            self.assertTrue(any('Points3D' in type(c.args[1]).__name__ for c in log.call_args_list))
