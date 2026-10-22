"""Pack-function and service-call tests. A fake node at most; no bridge, no network."""

import pytest

# Not importorskip: with the repo root on sys.path the source bridge_interface/
# directory shadows the built package as a namespace package, so the module
# imports and the symbols are missing.
try:
    from bridge_interface.msg import MotorCmd, RobotCmd
except ImportError:
    pytest.skip("source the colcon workspace first", allow_module_level=True)

from std_srvs.srv import Trigger  # noqa: E402

from robot_bridge import cmd_client  # noqa: E402
from robot_bridge.cmd_client import MOTOR_MODE, TOPIC, RobotCmdClient, pack_robot_cmd  # noqa: E402

N = 29

# float32 message fields, so exact equality does not survive the round trip.
REL = 1e-5


def arrays(n=N):
    return (
        [0.1 * i for i in range(n)],  # q
        [0.2 * i for i in range(n)],  # dq
        [0.3 * i for i in range(n)],  # tau
        [10.0 * i for i in range(n)],  # kp
        [0.5 * i for i in range(n)],
    )  # kd


def test_every_field_lands_on_its_motor():
    q, dq, tau, kp, kd = arrays()
    cmd = pack_robot_cmd(q, dq, tau, kp, kd)

    assert len(cmd.motor_cmd) == N
    for i, motor in enumerate(cmd.motor_cmd):
        assert motor.q == pytest.approx(q[i], rel=REL)
        assert motor.dq == pytest.approx(dq[i], rel=REL)
        assert motor.tau == pytest.approx(tau[i], rel=REL)
        assert motor.kp == pytest.approx(kp[i], rel=REL)
        assert motor.kd == pytest.approx(kd[i], rel=REL)
        assert motor.mode == MOTOR_MODE
        assert list(motor.reserve) == [0, 0, 0]


def test_header_fields_are_carried_through():
    cmd = pack_robot_cmd(*arrays(), interpolation_order=1.0, duration=0.02, hold_position=True)
    assert cmd.interpolation_order == 1.0
    assert cmd.duration == pytest.approx(0.02)
    assert cmd.hold_position is True


def test_header_defaults_are_linear_not_holding():
    cmd = pack_robot_cmd(*arrays())
    assert cmd.interpolation_order == 1.0  # a forgotten order ramps over `duration`, not a step
    assert cmd.hold_position is False


def test_numpy_input_is_accepted():
    """float64 assigned to a float32 field is what rosidl rejects; pack casts first."""
    np = pytest.importorskip("numpy")
    q = np.arange(N, dtype=np.float64) * 0.1
    cmd = pack_robot_cmd(q, q, q, q, q)
    assert len(cmd.motor_cmd) == N
    assert cmd.motor_cmd[1].q == pytest.approx(0.1, rel=REL)


def test_ragged_input_is_rejected():
    q, dq, tau, kp, kd = arrays()
    with pytest.raises(ValueError, match="equal length"):
        pack_robot_cmd(q, dq[:-1], tau, kp, kd)


def test_wire_contract_is_pinned():
    """Field order and widths must match bridge_interface/msg/*.msg."""
    assert TOPIC == "/robot_cmd"
    assert MotorCmd.get_fields_and_field_types() == {
        "mode": "uint8",
        "q": "float",
        "dq": "float",
        "tau": "float",
        "kp": "float",
        "kd": "float",
        "reserve": "uint32[3]",
    }
    assert RobotCmd.get_fields_and_field_types() == {
        "interpolation_order": "double",
        "hold_position": "boolean",
        "duration": "double",
        "motor_cmd": "sequence<bridge_interface/MotorCmd>",
    }


class _Future:
    def __init__(self, result):
        self._result = result

    def done(self):
        return True

    def result(self):
        return self._result


class _ServiceClient:
    def __init__(self, srv_name, up):
        self.srv_name = srv_name
        self.up = up
        self.requests = []

    def wait_for_service(self, timeout_sec=None):
        return self.up

    def call_async(self, request):
        self.requests.append(request)
        return _Future(f"reply from {self.srv_name}")


class _Node:
    def __init__(self, up=True):
        self.up = up
        self.clients = {}

    def create_publisher(self, *args, **kwargs):
        return object()

    def create_client(self, srv_type, name):
        self.clients[name] = _ServiceClient(name, self.up)
        return self.clients[name]


def test_ready_position_control_calls_its_service(monkeypatch):
    monkeypatch.setattr(cmd_client.rclpy, "spin_until_future_complete", lambda *a, **k: None)
    node = _Node()
    client = RobotCmdClient(node, num_dof=N, control_frequency=50.0)

    assert client.ready_position_control() == "reply from /ready_position_control"
    requests = node.clients["/ready_position_control"].requests
    assert len(requests) == 1 and isinstance(requests[0], Trigger.Request)


def test_ready_position_control_without_the_bridge_raises_like_stop_control():
    client = RobotCmdClient(_Node(up=False), num_dof=N, control_frequency=50.0)
    with pytest.raises(RuntimeError, match="/stop_control is not available"):
        client.stop_control(timeout_sec=0.0)
    with pytest.raises(RuntimeError, match="/ready_position_control is not available"):
        client.ready_position_control(timeout_sec=0.0)
