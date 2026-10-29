"""Rotation direction, IMU mount/waist compensation, and localization authority."""
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock, patch
import unittest
import numpy as np
from h1_2_viewer.attitude import unitree_attitude, matrix_quaternion
from h1_2_viewer.spatial_data import Pose
from h1_2_viewer.configuration import load_profile
from h1_2_viewer.model import RobotModel
from h1_2_viewer.spatial_renderer import SpatialRenderer
from h1_2_viewer.jsonl_source import dispatch

ROOT = Path(__file__).parents[1]


class AttitudeMath(unittest.TestCase):
    def test_imu_curves_units_and_rate_limit(self):
        from h1_2_viewer.renderer import Dashboard
        profile = load_profile()
        profile.settings['attitude']['enabled'] = True
        model = S(log_geometry=lambda: None, hand_reference_note='reference')
        with patch('rerun.log') as log:
            dashboard = Dashboard(model, profile)
            dashboard.spatial = Mock()
            log.reset_mock()
            for i in range(100):
                dashboard.attitude([0, 0, 0, 1], [0, np.pi/3, np.pi/2], elapsed=i/100)
            curves = [c for c in log.call_args_list if c.args[0].startswith('imu/')]
            self.assertEqual(len(curves), 6)  # Three batched curves, twice per second.
            np.testing.assert_allclose(curves[0].args[1].scalars.as_arrow_array().to_pylist(), [0, 60])
            np.testing.assert_allclose(curves[1].args[1].scalars.as_arrow_array().to_pylist(), [90])
            np.testing.assert_allclose(curves[2].args[1].scalars.as_arrow_array().to_pylist(), [0, 0, 0, 1])
            text = [c.args[1].text.as_arrow_array().to_pylist()[0] for c in log.call_args_list if c.args[0] == 'panels/attitude']
            self.assertFalse(any('np.float' in value for value in text))

    def test_wxyz_order_and_invalid_input(self):
        msg = S(imu_state=S(quaternion=[np.sqrt(.5), 0, np.sqrt(.5), 0], rpy=[0, np.pi/2, 0]))
        q, rpy = unitree_attitude(msg)
        np.testing.assert_allclose(Pose('world', [0, 0, 0], q).matrix()[:3, 0], [0, 0, -1], atol=1e-8)
        self.assertEqual(rpy, (0, np.pi/2, 0))
        for raw in ([0, 0, 0, 0], [float('nan'), 0, 0, 1]):
            msg.imu_state.quaternion = raw
            with self.assertRaises(ValueError): unitree_attitude(msg)

    def test_matrix_roundtrip_near_half_turn(self):
        for q in ([1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [.2, .3, .4, np.sqrt(.71)]):
            m = Pose('world', [0, 0, 0], q).matrix()
            actual = Pose('world', [0, 0, 0], matrix_quaternion(m[:3, :3])).matrix()
            np.testing.assert_allclose(actual, m, atol=1e-8)

    def test_jsonl_is_transport_independent(self):
        sink = Mock()
        dispatch(sink, dict(schema_version=1, type='attitude', quaternion_xyzw=[0, 0, 0, 1], rpy=[0, 0, 0], elapsed=1))
        sink.attitude.assert_called_once_with([0, 0, 0, 1], [0, 0, 0], elapsed=1)


class AttitudeModel(unittest.TestCase):
    def setUp(self):
        self.profile = load_profile(ROOT/'h1_2_viewer/config/h1_2_mid360.yaml')
        self.model = RobotModel(self.profile)
        self.addCleanup(self.model.close)
        self.log = patch('rerun.log').start()
        self.addCleanup(patch.stopall)
        self.spatial = SpatialRenderer(self.model, self.profile.settings['spatial'])
        self.state = {j.name: 0. for j in self.profile.joints}
        self.state['torso_joint'] = .3
        self.spatial.update_body(self.state)

    def test_compensates_waist_and_rotates_lidar_and_robot_together(self):
        # Synthetic known tilted pelvis + nonzero waist, not an assumed upright IMU.
        root = Pose('imu_world', [0, 0, 0], [.1, .2, .3, np.sqrt(.86)]).matrix()
        root_imu = self.model.kinematics.tip_matrix(self.state, 'imu_link', require_complete=True)
        q = matrix_quaternion((root @ root_imu)[:3, :3])
        self.assertTrue(self.spatial.attitude(q, 'imu_link'))
        np.testing.assert_allclose(self.spatial.graph.lookup('imu_world', self.spatial.robot_frame), root, atol=1e-7)
        lidar = self.model.kinematics.tip_matrix(self.state, 'lidar_link', require_complete=True)
        lidar = lidar @ np.diag([1, -1, -1, 1])
        np.testing.assert_allclose(self.spatial.graph.lookup('imu_world', 'livox_frame'), root @ lidar, atol=1e-7)
        # Relative sensor mounting must not change when the entire scene tilts.
        np.testing.assert_allclose(self.spatial.graph.lookup(self.spatial.robot_frame, 'livox_frame'), lidar, atol=1e-7)

    def test_localization_takes_over_and_imu_cannot_overwrite(self):
        self.spatial.attitude([0, 0, 0, 1], 'imu_link')
        self.spatial.robot_pose(Pose('imu_world', [1, 2, 3]))
        self.assertFalse(self.spatial.attitude([0, 0, 0, 1], 'imu_link'))
        np.testing.assert_allclose(self.spatial.graph.lookup('imu_world', self.spatial.robot_frame)[:3, 3], [1, 2, 3])

    def test_explicit_tf_can_replace_fallback_but_internal_links_stay_owned(self):
        self.spatial.attitude([0, 0, 0, 1], 'imu_link')
        self.spatial.tf(self.spatial.robot_frame, Pose('imu_world', [2, 0, 0]))
        self.assertFalse(self.spatial.attitude([0, 0, 0, 1], 'imu_link'))
        with self.assertRaises(ValueError):
            self.spatial.tf('lidar_link', Pose('imu_world', [0, 0, 0]))

    def test_blueprint_uses_world_origin_so_tilt_is_visible(self):
        from h1_2_viewer.layout import blueprint
        import rerun.blueprint as rrb
        with patch('h1_2_viewer.layout.rrb.Spatial3DView', wraps=rrb.Spatial3DView) as view:
            blueprint(self.profile)
            self.assertEqual(view.call_args.kwargs['origin'], '/scene')
