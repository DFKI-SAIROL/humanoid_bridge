"""RGB-D association and pixel correspondence, without a ROS runtime."""
import unittest
from types import SimpleNamespace as S
from unittest.mock import Mock

import numpy as np

from h1_2_viewer.rgbd import RgbdPairer
from h1_2_viewer.head_camera_ros import drain
from h1_2_viewer.configuration import load_profile
from h1_2_viewer.sensors import depth_points


def image(ns, frame='optical', width=2, height=2):
    sec, nanosec = divmod(ns, 1_000_000_000)
    return S(header=S(stamp=S(sec=sec, nanosec=nanosec), frame_id=frame),
             width=width, height=height)


class RgbdTests(unittest.TestCase):
    def test_exact_pair_both_arrival_orders_and_no_reuse(self):
        for first, second in [('rgb', 'depth'), ('depth', 'rgb')]:
            p = RgbdPairer()
            messages = {key: image(1_000_000_123) for key in ('rgb', 'depth')}
            self.assertIsNone(p.add(first, messages[first]))
            pair = p.add(second, messages[second])
            self.assertIs(pair[0], messages['depth'])
            self.assertIs(pair[1], messages['rgb'])
            self.assertIsNone(p.add(second, messages[second]))
            self.assertEqual(p.matched, 1)

    def test_tolerance_boundary_and_no_neighbour_frame_substitution(self):
        p = RgbdPairer(tolerance_ms=5)
        depth = image(1_000_000_000)
        self.assertIsNone(p.add('depth', depth))
        self.assertIsNone(p.add('rgb', image(1_005_000_001)))
        self.assertIsNotNone(p.add('rgb', image(1_005_000_000)))
        self.assertEqual(p.last_delta_ns, 5_000_000)
        p = RgbdPairer()
        p.add('depth', depth)
        self.assertIsNone(p.add('rgb', image(1_000_000_001)))

    def test_geometry_mismatch_and_bounded_missing_stream(self):
        p = RgbdPairer()
        p.add('depth', image(100))
        self.assertIsNone(p.add('rgb', image(100, frame='other')))
        self.assertIsNone(p.add('rgb', image(100, width=3)))
        for timestamp in range(200, 1200):
            p.add('depth', image(timestamp))
        self.assertLessEqual(len(p.queues['depth']), 4)
        self.assertLessEqual(len(p.queues['rgb']), 4)

    def test_out_of_order_late_data_cannot_rewind_pairs(self):
        p = RgbdPairer()
        p.add('depth', image(200))
        p.add('rgb', image(200))
        self.assertIsNone(p.add('depth', image(100)))
        self.assertIsNone(p.add('rgb', image(100)))
        self.assertEqual(p.matched, 1)

    def test_adapter_uses_paired_pixels_not_latest_rgb_and_masks_invalid_depth(self):
        rgb, depth = image(100), image(100)
        pixels = np.array([[[255, 0, 0], [0, 255, 0]],
                           [[0, 0, 255], [255, 255, 0]]], dtype=np.uint8)
        rgb.encoding, rgb.step, rgb.data = 'rgb8', 6, pixels.tobytes()
        depth.encoding, depth.is_bigendian, depth.step = '16UC1', False, 4
        depth.data = np.array([[1000, 0], [3000, 4000]], dtype='<u2').tobytes()
        info = S(header=depth.header, width=2, height=2, d=[0]*5, k=np.eye(3).ravel())
        latest = image(200)
        latest.encoding, latest.step, latest.data = 'rgb8', 6, bytes([255]*12)
        node = S(head_pair=(depth, rgb), head_pending=None, head_info=info, head_rgb_pending=latest)
        dashboard = Mock(errors={})
        profile = load_profile()
        drain(node, dashboard, profile)
        args, kwargs = dashboard.head_depth.call_args
        np.testing.assert_array_equal(kwargs['rgb'], pixels)
        xyz, colors = depth_points(args[0], args[1], kwargs['rgb'], stride=1)
        np.testing.assert_array_equal(colors, [[255, 0, 0], [0, 0, 255], [255, 255, 0]])
        np.testing.assert_allclose(xyz[:, 2], [1, 3, 4])
        self.assertIsNone(node.head_pair)

    def test_unpaired_color_still_displays_without_fake_colored_depth(self):
        rgb = image(100)
        rgb.encoding, rgb.step, rgb.data = 'rgb8', 6, bytes([255]*12)
        node = S(head_pair=None, head_pending=None, head_info=None, head_rgb_pending=rgb)
        dashboard = Mock(errors={})
        drain(node, dashboard, load_profile())
        dashboard.head_rgb.assert_called_once()
        dashboard.head_depth.assert_not_called()
