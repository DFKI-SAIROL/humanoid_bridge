"""RobotSim on the T1, no ROS. Needs mujoco and a T1 model (fetch_assets.sh t1_ncbf, t1_booster)."""

import pathlib

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

from robot_sim.physics import Command, RobotSim  # noqa: E402
from robot_sim.robots import t1  # noqa: E402

DATA = pathlib.Path("~/.local/share/robot_web_commander").expanduser()
MODELS = [DATA / "t1_ncbf" / "t1.xml", DATA / "t1_booster" / "T1_23dof.xml"]


@pytest.fixture(params=MODELS, ids=["ncbf", "booster"])
def sim(request):
    if not request.param.is_file():
        pytest.skip(f"no {request.param}")
    return RobotSim(t1.SPEC, request.param)


def hold(sim, target, kp=80.0, kd=2.0):
    cmd = Command(sim.num_joint)
    cmd.mode[:] = 1
    cmd.q[:], cmd.kp[:], cmd.kd[:] = target, kp, kd
    return cmd


def test_motor_order_is_the_bridge_order(sim):
    names = [sim.model.joint(sim.model.jnt_qposadr.tolist().index(a)).name for a in sim.qadr]
    assert names == list(t1.JOINT_NAMES) and sim.num_joint == 23


def test_the_head_camera_looks_forward_and_down(sim):
    mujoco.mj_forward(sim.model, sim.data)
    view = -sim.data.cam_xmat[sim.model.camera(t1.CAMERA).id].reshape(3, 3)[:, 2]  # along -z
    up = sim.data.cam_xmat[sim.model.camera(t1.CAMERA).id].reshape(3, 3)[:, 1]
    assert view[0] > 0.95 and view[2] < 0.0, view
    assert up[2] > 0.9, up


def test_the_gantry_holds_a_limp_t1_level(sim):
    for _ in range(int(2.0 / sim.dt)):
        sim.step(Command(sim.num_joint))
    s = sim.state()
    assert abs(s["rpy"][0]) < 0.05 and abs(s["rpy"][1]) < 0.05
    assert sim.data.qpos[2] == pytest.approx(sim.anchor[2], abs=0.05)


def test_pd_holds_a_pose_within_the_effort(sim):
    target = np.zeros(sim.num_joint)
    target[t1.JOINT_NAMES.index("left_elbow_pitch_joint")] = 0.5
    cmd = hold(sim, target)
    for _ in range(int(2.0 / sim.dt)):
        sim.step(cmd)
    s = sim.state()
    assert np.abs(s["q"] - target).max() < 0.1
    assert np.all(np.abs(s["tau_est"]) <= sim.effort + 1e-9)


def test_mode_requests_and_buttons():
    assert t1.mode_request('{"mode":3}') == t1.CUSTOM
    assert t1.mode_request('{"mode":9}') is None
    assert t1.mode_request("not json") is None
    assert set(t1.PAD) >= {"RB", "RT", "LB"}
