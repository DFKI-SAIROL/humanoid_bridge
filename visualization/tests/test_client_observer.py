"""Control and visualization remain separate; targets never become measured pose."""
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock, patch
import ast
import unittest
import numpy as np
from h1_2_viewer.client_observer import body_target, drain
from h1_2_viewer.configuration import Joint, load_profile
from h1_2_viewer.renderer import Dashboard
from h1_2_viewer.jsonl_source import dispatch


class ClientObserver(unittest.TestCase):
    def test_motor_order_mapping_and_validation(self):
        joints=[Joint('a',1,'arm',-1,.2),Joint('b',0,'arm')]
        np.testing.assert_allclose(body_target(S(motor_cmd=[S(q=.3),S(q=.4)]),joints),[-.2,.3])
        for motors in ([S(q=0)], [S(q=0),S(q=float('nan'))]):
            with self.assertRaises(ValueError):body_target(S(motor_cmd=motors),joints)

    def test_target_render_does_not_update_measured_model(self):
        dashboard=Dashboard.__new__(Dashboard)
        dashboard.names=('a',);dashboard.groups={'arm':[0]}
        dashboard.profile=S(settings={'source':{'hand_channels':12},'runtime':{'debug_hz':2}})
        dashboard.timestamp=Mock();dashboard.accept=Mock();dashboard.model=Mock();dashboard.spatial=Mock()
        with patch('h1_2_viewer.renderer.rr.log'):
            dashboard.body_target([.4]);dashboard.hand_target([1]*12)
        dashboard.model.log_body.assert_not_called()
        dashboard.spatial.update_body.assert_not_called()
        dashboard.accept.assert_any_call('body_target')
        with self.assertRaises(ValueError):dashboard.body_target([float('nan')])

    def test_jsonl_and_latest_mailbox_dispatch(self):
        dashboard=Mock();dashboard.errors={}
        dispatch(dashboard,{'schema_version':1,'type':'body_target','q':[.2],'elapsed':1})
        dashboard.body_target.assert_called_once_with([.2],elapsed=1)
        profile=S(joints=[Joint('a',0,'arm')])
        node=S(client_pending={'body_target':S(motor_cmd=[S(q=.4)])})
        drain(node,dashboard,profile)
        self.assertEqual(node.client_pending,{})

    def test_viewer_has_no_control_entry_points(self):
        root=Path(__file__).parents[1]/'h1_2_viewer'
        for p in root.glob('*.py'):
            tree=ast.parse(p.read_text())
            for n in ast.walk(tree):
                if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute):
                    self.assertNotIn(n.func.attr,{'create_publisher','create_client','send_hand_cmd','send_cmd'})
                if isinstance(n,ast.ImportFrom):
                    self.assertFalse((n.module or '').startswith(('whole_body_controller','robot_bridge_py')))

    def test_observer_config(self):
        p=load_profile(Path(__file__).parents[1]/'h1_2_viewer/config/client_observer.yaml')
        self.assertEqual(p.settings['source']['body_target_topic'],'/robot_cmd')
