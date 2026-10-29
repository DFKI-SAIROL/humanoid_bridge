"""Offline tests run with only viewer dependencies; no ROS imports or robot nodes."""
import ast
from dataclasses import replace
import io
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
from types import SimpleNamespace as S
import unittest
from unittest.mock import patch

import numpy as np
import yaml

from h1_2_viewer import assets
from h1_2_viewer.configuration import load_profile, Joint
from h1_2_viewer.model import RobotModel
from h1_2_viewer.pointcloud import cloud_xyz
from h1_2_viewer.telemetry import unitree_body, named_body
from h1_2_viewer.jsonl_source import dispatch
from h1_2_viewer.extensions import DashboardExtensions
from h1_2_viewer.renderer import Dashboard

ROOT = Path(__file__).resolve().parents[1]
CUSTOM = ROOT / 'models/custom/h1_2/viewer.yaml'


class CoreTests(unittest.TestCase):
    def test_relative_config_is_independent_of_cwd(self):
        with tempfile.TemporaryDirectory(prefix='viewer profile ') as d:
            path = Path(d) / 'settings.yaml'
            path.write_text('model:\n  urdf: model/custom.urdf\n  package_paths:\n    hand: resources/hand\n')
            profile = load_profile(path)
            self.assertEqual(profile.urdf, path.parent / 'model/custom.urdf')
            self.assertEqual(profile.package_paths['hand'], path.parent / 'resources/hand')

    def test_invalid_profiles_rejected(self):
        for text in ('runtime:\n  hz: 0\n', 'runtime:\n  hzz: 20\n', 'schema_version: 99\n',
                     'joints:\n- {name: x, motor_index: 0, group: a}\n- {name: x, motor_index: 1, group: a}\n'):
            with self.subTest(text=text), tempfile.TemporaryDirectory() as d:
                path = Path(d) / 'bad.yaml'
                path.write_text(text)
                with self.assertRaises(ValueError):
                    load_profile(path)

    def test_motor_mapping_reorder_scale_offset(self):
        motors = [S(q=1, dq=2, tau_est=3), S(q=4, dq=5, tau_est=6)]
        mapping = [Joint('b', 1, 'arm', -1, 0.25), Joint('a', 0, 'leg')]
        np.testing.assert_allclose(unitree_body(S(motor_state=motors), mapping),
                                   [[-3.75, 1], [-5, 2], [6, 3]])
        with self.assertRaises(ValueError):
            unitree_body(S(motor_state=[]), mapping)

    def test_named_joint_state_does_not_apply_hardware_mapping_twice(self):
        mapping = [Joint('a', 20, 'arm', -1, 2), Joint('b', 21, 'arm')]
        msg = S(name=['b', 'a'], position=[1, 2], velocity=[], effort=[])
        values = named_body(msg, mapping)
        np.testing.assert_allclose(values[0], [2, 1])
        self.assertTrue(np.isnan(values[1:]).all())
        msg.name = ['a', 'a']
        with self.assertRaises(ValueError):
            named_body(msg, mapping)

    def test_padded_big_endian_pointcloud(self):
        msg = S(height=2, width=1, point_step=16, row_step=20, is_bigendian=True,
                fields=[S(name=n, offset=i * 4, datatype=7, count=1) for i, n in enumerate('xyz')],
                data=struct.pack('>fff', 1, 2, 3) + bytes(8) + struct.pack('>fff', 4, 5, 6) + bytes(8))
        np.testing.assert_allclose(cloud_xyz(msg), [[1, 2, 3], [4, 5, 6]])
        msg.data = msg.data[:4]
        with self.assertRaises((ValueError, TypeError)):
            cloud_xyz(msg)

    @patch('rerun.log')
    @patch('rerun.set_time')
    def test_invalid_samples_and_staleness(self, time, log):
        profile = load_profile()
        model = S(log_geometry=lambda: None, hand_reference_note='reference',
                  log_body=lambda q: None)
        dashboard = Dashboard(model, profile)
        log.reset_mock()
        with self.assertRaises(ValueError):
            dashboard.body([float('nan')] * 27, [0] * 27, [0] * 27)
        log.assert_not_called()
        dashboard.last['body'] = dashboard.started - 10
        dashboard.status()
        self.assertIn('STALE', str(log.call_args))

    @patch('rerun.log')
    def test_extension_frame_contract(self, log):
        with self.assertRaises(ValueError):
            DashboardExtensions().log_path([[0, 0, 0]], frame_id='lidar')

    def test_jsonl_schema_routes_without_command_interface(self):
        from unittest.mock import Mock
        sink = Mock()
        dispatch(sink, {'schema_version': 1, 'type': 'body', 'q': [1], 'dq': [2], 'tau': [3], 'elapsed': 0.5})
        sink.body.assert_called_once_with([1], [2], [3], elapsed=0.5)
        with self.assertRaises(ValueError):
            dispatch(sink, {'schema_version': 1, 'type': 'execute_policy'})
        with self.assertRaises(ValueError):
            dispatch(sink, {'schema_version': 99, 'type': 'body'})

    def test_offline_import_does_not_load_ros_or_controller(self):
        script = '''import sys
from h1_2_viewer.cli import main
main(['--help'])
'''
        # Block import attempts, rather than relying on ROS being absent on this machine.
        script = '''import builtins
real_import = builtins.__import__
def checked(name, *args, **kwargs):
    if name.split('.')[0] in ('rclpy', 'sensor_msgs', 'unitree_hg', 'unitree_go', 'robot_bridge_py', 'whole_body_controller'):
        raise AssertionError('Offline viewer attempted ROS/controller import: ' + name)
    return real_import(name, *args, **kwargs)
builtins.__import__ = checked
''' + script
        result = subprocess.run([sys.executable, '-c', script], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_full_demo_is_ros_free(self):
        with tempfile.TemporaryDirectory(prefix='standalone demo ') as d:
            root = Path(d)
            (root / 'robot.urdf').write_text('<robot name="tiny"><link name="base"><visual><geometry><box size="1 1 1"/></geometry></visual></link><link name="child"/><joint name="hinge" type="revolute"><parent link="base"/><child link="child"/><axis xyz="0 0 1"/><limit lower="-2" upper="2" effort="1" velocity="1"/></joint></robot>')
            profile = root / 'viewer.yaml'
            profile.write_text(yaml.safe_dump({'model': {'urdf': 'robot.urdf'}, 'joints': [{'name': 'hinge', 'motor_index': 0, 'group': 'arm'}]}))
            output = root / 'demo.rrd'
            script = """import builtins,sys
original = builtins.__import__
def isolated(name, *args, **kwargs):
    if name.split('.')[0] in ('rclpy','sensor_msgs','unitree_hg','unitree_go','robot_bridge_py','whole_body_controller'):
        raise AssertionError('Forbidden offline import: ' + name)
    return original(name,*args,**kwargs)
builtins.__import__=isolated
from h1_2_viewer.cli import main
main(sys.argv[1:])
"""
            result = subprocess.run([sys.executable, '-c', script, '--config', str(profile), '--demo', '--demo-frames', '2', '--save', str(output)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertGreater(output.stat().st_size, 0)

    def test_demo_visits_every_stage_and_only_confirms_at_end(self):
        from h1_2_viewer.demo import phase
        from h1_2_viewer.task import STAGES, TaskState
        snapshots = [phase(value)[2] for value in np.linspace(0, 1, 420)]
        self.assertEqual(set(state.stage for state in snapshots), {key for key, _ in STAGES})
        self.assertEqual(snapshots[0].distance, 2.4)
        self.assertFalse(snapshots[0].target_visible)
        self.assertFalse(any(state.grasp_confirmed for state in snapshots if state.stage != 'confirm'))
        self.assertTrue(snapshots[-1].grasp_confirmed)
        self.assertEqual(snapshots[-1].progress, 1)
        self.assertIn('SIMULATED SUCCESS', snapshots[-1].markdown())
        with self.assertRaises(ValueError):
            TaskState('grasp', .5, 0, grasp_confirmed=True)

    @patch('rerun.log')
    @patch('rerun.set_time')
    def test_task_and_camera_validation(self, time, log):
        from h1_2_viewer.task import TaskState
        profile = load_profile()
        model = S(log_geometry=lambda: None, hand_reference_note='reference')
        dashboard = Dashboard(model, profile)
        placeholders = [call for call in log.call_args_list if call.args[0] == 'panels/task' or call.args[0].endswith('/waiting')]
        self.assertTrue(placeholders)
        self.assertTrue(all(not call.kwargs.get('static', False) for call in placeholders))
        log.reset_mock()
        with self.assertRaises(ValueError):
            dashboard.camera('left', np.zeros((2, 2, 3), dtype=np.uint8), np.zeros((2, 2)))
        log.assert_not_called()
        dashboard.camera('right', np.zeros((2, 2, 3), dtype=np.uint8), np.ones((2, 2)))
        dashboard.task(TaskState('observe', .4, 0, target_visible=True, simulated=True))
        self.assertIn('Observe target', str(log.call_args))

    def test_exporter_has_no_rerun_import(self):
        tree = ast.parse((ROOT / 'tools/ros2_export.py').read_text())
        imports = [node.module for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)]
        self.assertNotIn('rerun', imports)
        import importlib.util
        spec = importlib.util.spec_from_file_location('exporter', ROOT / 'tools/ros2_export.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        output = io.StringIO()
        module.JsonlSink(output).body(np.array([1.]), np.array([float('nan')]), np.array([2.]))
        event = json.loads(output.getvalue())
        self.assertEqual(event['dq'], [None])
        self.assertEqual(event['schema_version'], 1)

    def test_geometry_primitives_package_uri_and_mimic(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            # No mesh is necessary: official loader supports URDF primitives too.
            urdf = root / 'custom.urdf'
            urdf.write_text('''<robot name="test"><link name="base"><visual><geometry><box size="1 1 1"/></geometry></visual></link><link name="child"/><link name="finger"/><joint name="hinge" type="revolute"><parent link="base"/><child link="child"/><axis xyz="0 0 1"/><limit lower="-2" upper="2" effort="1" velocity="1"/></joint><joint name="mimic" type="revolute"><parent link="child"/><child link="finger"/><axis xyz="0 1 0"/><limit lower="-2" upper="2" effort="1" velocity="1"/><mimic joint="hinge" multiplier="0.5" offset="0.1"/></joint></robot>''')
            profile = replace(load_profile(), urdf=urdf, joints=(Joint('hinge', 0, 'arm'),), package_paths={'custom': root})
            resource = root / 'resource.STL'
            resource.write_bytes(b'dummy resource path test')
            model = RobotModel(profile)
            try:
                self.assertEqual(model.resolve_resource('package://custom/resource.STL'), resource)
                with self.assertRaises(ValueError):
                    model.resolve_resource('package://unknown/resource.STL')
                with patch('rerun.log') as log:
                    model.log_body([0.4])
                    self.assertEqual(log.call_count, 2)
                    self.assertEqual(log.call_args.args[0], 'robot/transforms/mimic')
            finally:
                model.close()

    def test_model_fetch_and_fork_never_overwrite_edits(self):
        # Tiny synthetic lock exercises atomic copy/hash verification without network or 80MB fixtures.
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            source = root / 'source'
            (source / 'robots/model').mkdir(parents=True)
            (source / 'robots/model/a.urdf').write_text('original')
            (source / 'LICENSE').write_text('license')
            import hashlib
            lock = {'repository': 'unused', 'commit': 'fixed', 'directory': 'robots/model',
                    'license': 'test', 'sha256': {str(p.relative_to(source)): hashlib.sha256(p.read_bytes()).hexdigest()
                                                for p in source.rglob('*') if p.is_file()}}
            with patch('h1_2_viewer.assets.model_lock', return_value=lock):
                vendor = assets.fetch(root / 'vendor', source)
                custom = assets.fork(root / 'custom', vendor)
                (custom / 'h1_2_description/a.urdf').write_text('edited')
                with self.assertRaises(FileExistsError):
                    assets.fork(custom, vendor)
                assets.verify(vendor)
                with self.assertRaises(ValueError):
                    assets.verify(custom)


@unittest.skipUnless(CUSTOM.is_file(), 'Official model integration requires an editable local fork')
class ModelIntegrationTests(unittest.TestCase):
    def test_model_hash_mapping_and_source_immutability(self):
        assets.verify(ROOT / 'models/upstream')
        profile = load_profile(CUSTOM)
        original = profile.urdf.read_bytes()
        model = RobotModel(profile)
        try:
            self.assertEqual(len(profile.joints), 27)
            self.assertGreater(len(model.meshes), 40)
            self.assertEqual(model.tree.root_link().name, 'pelvis')
            self.assertEqual(profile.urdf.read_bytes(), original)
            bridge = ROOT.parent / 'robot_bridge/params/H1_2_config.yaml'
            if bridge.is_file():
                params = yaml.safe_load(bridge.read_text())['robot_bridge']['ros__parameters']
                names = ['WAIST_YAW' if j.name == 'torso_joint' else j.name.removesuffix('_joint').upper() for j in profile.joints]
                self.assertEqual(names, params['joint_names'])
                self.assertEqual([j.motor_index for j in profile.joints], list(range(27)))
        finally:
            model.close()


if __name__ == '__main__':
    unittest.main()
