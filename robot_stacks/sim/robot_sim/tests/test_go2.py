"""RobotSim on the Go2, no ROS. Needs mujoco and a Go2 model (fetch_assets.sh go2_ncbf, ...)."""

import pathlib
import random

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

from robot_sim.physics import Command, RobotSim  # noqa: E402
from robot_sim.robots import go2  # noqa: E402

DATA = pathlib.Path("~/.local/share/robot_web_commander").expanduser()
MODELS = [DATA / "go2_ncbf" / "go2.xml", DATA / "go2_menagerie" / "go2.xml"]
STAND = [-0.1, 0.8, -1.5, 0.1, 0.8, -1.5, -0.1, 0.8, -1.5, 0.1, 0.8, -1.5]  # ready_q


@pytest.fixture(params=MODELS, ids=["ncbf", "menagerie"])
def sim(request):
    if not request.param.is_file():
        pytest.skip(f"no {request.param}")
    return RobotSim(go2.SPEC, request.param, clearance=0.0)


def run(sim, cmd, seconds):
    for _ in range(int(seconds / sim.dt)):
        sim.step(cmd)
    return sim.state()


def test_motor_order_is_the_bridge_order(sim):
    names = [sim.model.joint(sim.model.jnt_qposadr.tolist().index(a)).name for a in sim.qadr]
    assert names == list(go2.JOINT_NAMES) and sim.num_joint == 12


def test_the_ego_camera_looks_forward(sim):
    mujoco.mj_forward(sim.model, sim.data)
    view = -sim.data.cam_xmat[sim.model.camera(go2.CAMERA).id].reshape(3, 3)[:, 2]
    assert view[0] > 0.9, view


def test_it_starts_lying_and_a_limp_go2_stays_down(sim):
    assert sim.robot.gantry is False
    assert np.allclose(sim.state()["q"], go2.LYING_Q)
    start_z = sim.data.qpos[2]
    s = run(sim, Command(sim.num_joint), 2.0)  # limp: the hips splay, nothing pulls them back
    assert sim.data.qpos[2] <= start_z + 0.01, "it settles; nothing lifts it"
    assert np.abs(s["dq"]).max() < 0.5, "at rest: the model's own actuators are off"


def test_pd_stands_it_up_from_lying(sim):
    cmd = Command(sim.num_joint)
    cmd.mode[:] = 1
    cmd.q[:], cmd.kp[:], cmd.kd[:] = STAND, 70.0, 3.0  # Go2_config.yaml: the stand gains
    s = run(sim, cmd, 3.0)
    assert np.abs(s["q"] - STAND).max() < 0.25, "the calves sag under the body's weight"
    assert sim.data.qpos[2] > 0.2, "standing"
    assert np.all(np.abs(s["tau_est"]) <= sim.effort + 1e-9)


def test_the_crc_is_unitree_crc32_core():
    def core(words):
        crc = 0xFFFFFFFF
        for word in words:
            bit = 1 << 31
            for _ in range(32):
                crc = ((crc << 1) & 0xFFFFFFFF) ^ (0x04C11DB7 if crc & 0x80000000 else 0)
                if word & bit:
                    crc ^= 0x04C11DB7
                bit >>= 1
        return crc

    words = [random.Random(7).getrandbits(32) for _ in range(202)]
    assert go2.crc32_words(words) == core(words)
