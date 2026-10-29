"""Live display TF baseline, point budget and logging-rate regression checks."""
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import patch, Mock
import unittest
import numpy as np
from h1_2_viewer.configuration import load_profile
from h1_2_viewer.model import RobotModel
from h1_2_viewer.renderer import Dashboard
from h1_2_viewer.pointcloud import display_cloud

ROOT=Path(__file__).parents[1]


class LiveDisplay(unittest.TestCase):
    def test_voxel_filter_and_hard_budget(self):
        rng=np.random.default_rng(7)
        points=rng.uniform(-10,10,(20064,3)).astype(np.float32)
        reduced=display_cloud(points,max_points=4000,voxel_size=.04)
        self.assertLessEqual(len(reduced),4000)
        self.assertEqual(reduced.dtype,np.float32)
        small=display_cloud([[0,0,0],[.001,0,0],[1,0,0]],max_points=4000,voxel_size=.04)
        self.assertEqual(len(small),2)
        self.assertEqual(display_cloud(np.empty((0,3))).shape,(0,3))

    def test_live_curves_are_throttled_and_grouped(self):
        profile=load_profile()
        model=S(log_geometry=lambda:None,log_body=Mock(),hand_reference_note='reference')
        with patch('h1_2_viewer.renderer.rr.log') as log:
            dashboard=Dashboard(model,profile,demo=False)
            log.reset_mock()
            for i in range(100):
                dashboard.body([0]*27,[0]*27,[0]*27,elapsed=i/100)
            scalar_calls=[c for c in log.call_args_list if c.args[0].startswith('joints/')]
            # Two 5-group × 3-metric updates in one second, instead of 8100 calls.
            self.assertEqual(len(scalar_calls),30)
            self.assertEqual(dashboard.counts['body'],100)
            self.assertEqual(model.log_body.call_count,100)

    def test_lidar_sends_one_bounded_layer_and_keeps_freshness(self):
        profile=load_profile()
        model=S(log_geometry=lambda:None,log_body=Mock(),hand_reference_note='reference')
        with patch('h1_2_viewer.renderer.rr.log') as log:
            dashboard=Dashboard(model,profile)
            dashboard.sensors=Mock()
            for i in range(10):dashboard.cloud(np.ones((20064,3)),'livox_frame',elapsed=i/10)
            self.assertEqual(dashboard.sensors.cloud.call_count,5)
            self.assertEqual(dashboard.counts['lidar'],10)
            self.assertFalse(any(c.args[0]=='lidar_raw/points' for c in log.call_args_list))
            self.assertLessEqual(len(dashboard.sensors.cloud.call_args.args[1]),4000)

    def test_collision_meshes_not_loaded_or_source_modified(self):
        import xml.etree.ElementTree as ET
        profile=load_profile(ROOT/'h1_2_viewer/config/h1_2_mid360.yaml')
        original=profile.urdf.read_bytes()
        model=RobotModel(profile)
        try:
            resolved=ET.parse(Path(model._temp.name)/'resolved.urdf').getroot()
            self.assertEqual(resolved.findall('.//collision'),[])
            self.assertEqual(profile.settings['model']['mesh_face_limit'],0)
            self.assertFalse(list(Path(model._temp.name).glob('mesh_*.stl')))
            source_meshes=[m.get('filename') for m in resolved.findall('.//visual/geometry/mesh')]
            self.assertTrue(all(Path(m).is_relative_to(profile.urdf.parent) for m in source_meshes))
            self.assertEqual(profile.urdf.read_bytes(),original)
        finally:model.close()

    def test_official_mount_tf_composes_with_measured_torso(self):
        from h1_2_viewer.spatial_renderer import SpatialRenderer
        profile=load_profile(ROOT/'h1_2_viewer/config/h1_2_mid360.yaml')
        model=RobotModel(profile)
        try:
            with patch('rerun.log'):
                spatial=SpatialRenderer(model,profile.settings['spatial'])
                state={j.name:0. for j in profile.joints};state['torso_joint']=.3
                spatial.update_body(state)
                actual=spatial.graph.lookup(spatial.robot_frame,'livox_frame')
                expected=model.kinematics.tip_matrix(state,'lidar_link',require_complete=True)
                expected=expected@np.diag([1,-1,-1,1])
                np.testing.assert_allclose(actual,expected,atol=1e-7)
        finally:model.close()

    def test_measured_lidar_gravity_agrees_with_body_after_mount_correction(self):
        from h1_2_viewer.spatial_renderer import SpatialRenderer
        profile=load_profile(ROOT/'h1_2_viewer/config/h1_2_mid360.yaml')
        model=RobotModel(profile)
        try:
            with patch('rerun.log'):
                spatial=SpatialRenderer(model,profile.settings['spatial'])
                spatial.update_body({j.name:0. for j in profile.joints})
                # Independent near-static sample, 2026-10-29; compare direction, not driver units.
                raw=np.array([-.2629977346,.0092927886,-.9647516964])
                expected=np.array([-.0341667353,-.0139437837,.9993188706])
                transform=spatial.graph.lookup(spatial.resolve('torso_link'),'livox_frame')
                actual=transform[:3,:3]@raw
                cosine=np.dot(actual,expected)/np.linalg.norm(actual)/np.linalg.norm(expected)
                self.assertLess(np.degrees(np.arccos(np.clip(cosine,-1,1))),1.)
        finally:model.close()
