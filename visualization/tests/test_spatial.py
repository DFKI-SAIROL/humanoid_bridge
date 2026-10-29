"""Spatial contracts, TF composition and preview isolation; no ROS initialization."""
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import patch, Mock
import unittest
import numpy as np

from h1_2_viewer.spatial_data import Pose, FrameGraph, ArmTrajectory
from h1_2_viewer.spatial_ros import pose_message, transform_message, trajectory_message
from h1_2_viewer.jsonl_source import dispatch
from h1_2_viewer.configuration import load_profile
from h1_2_viewer.model import RobotModel
from h1_2_viewer.spatial_renderer import SpatialRenderer


class SpatialContracts(unittest.TestCase):
    def test_pose_validation_and_rotated_tf_chain(self):
        graph = FrameGraph()
        graph.set('odom', Pose('map', [1, 2, 0], [0, 0, np.sqrt(.5), np.sqrt(.5)]), static=True)
        graph.set('base', Pose('odom', [1, 0, 0]))
        np.testing.assert_allclose(graph.lookup('map', 'base')[:3, 3], [1, 3, 0], atol=1e-7)
        np.testing.assert_allclose(graph.lookup('base', 'map') @ graph.lookup('map', 'base'), np.eye(4), atol=1e-7)
        for quaternion in ([0, 0, 0, 0], [1, 2, 3, 4], [0, 0, float('nan'), 1]):
            with self.assertRaises(ValueError):
                Pose('map', [0, 0, 0], quaternion)
        with self.assertRaises(ValueError):
            Pose('', [0, 0, 0])

    def test_tf_conflicts_cycles_and_out_of_order_do_not_mutate(self):
        graph = FrameGraph()
        graph.set('odom', Pose('map', [0, 0, 0]), static=True)
        graph.set('base', Pose('odom', [0, 0, 0]), elapsed=2)
        for child, pose, options in [
            ('map', Pose('base', [0, 0, 0]), {}),
            ('base', Pose('map', [0, 0, 0]), {}),
            ('odom', Pose('map', [1, 0, 0]), {'static': True}),
            ('base', Pose('odom', [1, 0, 0]), {'elapsed': 1}),
        ]:
            with self.assertRaises(ValueError):
                graph.set(child, pose, **options)
        np.testing.assert_allclose(graph.lookup('map', 'base'), np.eye(4))
        with self.assertRaises(ValueError):
            graph.lookup('map', 'camera_without_tf')

    def test_trajectory_validation(self):
        base = dict(trajectory_id='plan', joint_names=['a'], positions=[[0], [.2]],
                    time_from_start=[0, 1], frame_id='pelvis', tip_link='wrist')
        ArmTrajectory(**base)
        for update in ({'time_from_start': [1, 0]}, {'time_from_start': [0, 0]},
                       {'positions': [[0], [float('nan')]]}, {'joint_names': ['a', 'a']},
                       {'trajectory_id': '../unsafe'}):
            with self.assertRaises(ValueError):
                ArmTrajectory(**{**base, **update})

    def test_jsonl_spatial_events_route_to_read_only_methods(self):
        sink = Mock()
        pose = dict(frame_id='map', position=[1, 2, 0], quaternion_xyzw=[0, 0, 0, 1])
        for kind, extra in [('robot_pose', {}), ('tf', {'child_frame': 'odom', 'static': True}),
                            ('object_pose', {'object_id': 'cup', 'size': [.1, .1, .2]}),
                            ('grasp_pose', {'grasp_id': 'right'})]:
            dispatch(sink, dict(schema_version=1, type=kind, pose=pose, elapsed=1, **extra))
        sink.robot_pose.assert_called_once()
        sink.tf.assert_called_once()
        sink.object_pose.assert_called_once()
        sink.grasp_pose.assert_called_once()
        dispatch(sink, dict(schema_version=1, type='arm_trajectory', trajectory=dict(
            trajectory_id='plan', joint_names=['a'], positions=[[0], [.1]], time_from_start=[0, 1],
            frame_id='pelvis', tip_link='wrist')))
        sink.arm_trajectory.assert_called_once()

    def test_ros_message_conversion_needs_no_ros_import(self):
        pose = S(position=S(x=1, y=2, z=3), orientation=S(x=0, y=0, z=0, w=1))
        self.assertEqual(pose_message(pose, 'camera').position, (1., 2., 3.))
        tf = S(header=S(frame_id='map'), child_frame_id='odom',
               transform=S(translation=S(x=1, y=0, z=0), rotation=pose.orientation))
        self.assertEqual(transform_message(tf)[0], 'odom')
        msg = S(header=S(frame_id=''), joint_names=['a'], points=[
            S(positions=[0], time_from_start=S(sec=0, nanosec=0)),
            S(positions=[1], time_from_start=S(sec=1, nanosec=500000000))])
        result = trajectory_message(msg, {'trajectory_id': 'p', 'tip_link': 'wrist'}, 'pelvis')
        self.assertEqual(result.time_from_start, (0, 1.5))
        self.assertEqual(result.frame_id, 'pelvis')


@unittest.skipUnless((Path(__file__).resolve().parents[1] / 'models/custom/h1_2/viewer.yaml').exists(), 'model not downloaded')
class SpatialModelIntegration(unittest.TestCase):
    def setUp(self):
        self.model = RobotModel(load_profile())
        self.logging = patch('rerun.log')
        self.logging.start()
        self.addCleanup(self.logging.stop)
        self.addCleanup(self.model.close)
        self.spatial = SpatialRenderer(self.model)

    def test_external_tf_cannot_override_urdf_joints(self):
        with self.assertRaises(ValueError):
            self.spatial.tf('left_knee_link', Pose('map', [0, 0, 0]))

    def test_camera_object_waits_then_resolves_with_robot_pose(self):
        self.spatial.tf('right_camera', Pose('right_wrist_yaw_link', [.04, 0, 0]), static=True)
        self.spatial.object_pose('cup', Pose('right_camera', [0, 0, .4]), size=[.1, .1, .2])
        self.assertTrue(self.spatial.pending)  # No map->pelvis localization yet.
        self.spatial.tf('odom', Pose('map', [.2, 0, 0]), static=True)
        self.spatial.robot_pose(Pose('odom', [1, 0, 1]))
        self.assertTrue(self.spatial.pending)  # A URDF reference pose is not measured wrist TF.
        self.spatial.update_body({j.name: 0 for j in self.model.profile.joints})
        self.assertFalse(self.spatial.pending)
        position = self.spatial.graph.lookup('map', self.spatial.robot_frame)[:3, 3]
        np.testing.assert_allclose(position, [1.2, 0, 1])

    def test_planned_joint_trajectory_does_not_change_measured_robot(self):
        names = tuple(j.name for j in self.model.profile.joints)
        measured = dict.fromkeys(names, 0.)
        self.spatial.update_body(measured)
        before = self.spatial.graph.lookup(self.spatial.robot_frame, 'h1_2/right_wrist_yaw_link').copy()
        q = np.zeros((2, len(names)))
        q[1, names.index('right_elbow_joint')] = .6
        self.spatial.arm_trajectory(ArmTrajectory('arm', names, q, [0, 2], 'pelvis', 'right_wrist_yaw_link'))
        np.testing.assert_allclose(before, self.spatial.graph.lookup(self.spatial.robot_frame, 'h1_2/right_wrist_yaw_link'))
        self.assertEqual(self.spatial.measured, measured)
        self.assertEqual(self.spatial.trajectories['arm'][1].shape, (2, 3))
        with self.assertRaises(ValueError):
            self.spatial.arm_trajectory(ArmTrajectory('bad', ['no_such_joint'], [[0], [.1]], [0, 1], 'pelvis', 'right_wrist_yaw_link'))
        self.assertNotIn('bad', self.spatial.trajectories)

    def test_missing_chain_state_is_not_fabricated(self):
        with self.assertRaises(ValueError):
            self.spatial.arm_trajectory(ArmTrajectory('incomplete', ['right_elbow_joint'], [[0], [.1]], [0, 1], 'pelvis', 'right_wrist_yaw_link'))
        self.assertFalse(self.spatial.trajectories)
