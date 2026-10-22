"""G1Sim without ROS. Needs mujoco and the mjlab g1.xml (fetch_assets.sh); skips without them."""

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

from g1_sim.model import (  # noqa: E402
    DEFAULT_MJCF,
    JOINT_NAMES,
    NUM_JOINT,
    Command,
    G1Sim,
    rpy_from_quat,
)

pytestmark = pytest.mark.skipif(not DEFAULT_MJCF.is_file(), reason=f"no {DEFAULT_MJCF}")

KP = np.array([100, 100, 100, 150, 40, 40] * 2 + [300] * 3 + [100, 100, 50, 50, 20, 20, 20] * 2)
KD = np.array([2, 2, 2, 4, 2, 2] * 2 + [5] * 3 + [2, 2, 2, 2, 1, 1, 1] * 2)


@pytest.fixture
def sim():
    return G1Sim()


def hold(target):
    cmd = Command()
    cmd.mode[:] = 1
    cmd.kp[:], cmd.kd[:], cmd.q[:] = KP, KD, target
    return cmd


def run(sim, cmd, seconds):
    for _ in range(int(seconds / sim.dt)):
        sim.step(cmd)
    return sim.state()


def test_motor_order_is_the_bridge_order(sim):
    hinges = [sim.model.joint(i).name for i in range(sim.model.njnt) if sim.model.jnt_type[i] == 3]
    assert hinges == list(JOINT_NAMES)
    assert NUM_JOINT == 29


def test_timestep_is_the_lowstate_period(sim):
    assert sim.dt == pytest.approx(0.002)


def test_d435_camera_is_on_the_torso(sim):
    cam = sim.model.camera("d435")
    assert sim.model.body(cam.bodyid[0]).name == "d435_link"


def test_gantry_holds_a_limp_robot_level(sim):
    s = run(sim, Command(), 2.0)
    assert np.abs(sim.data.qpos[:3] - sim.anchor).max() < 0.02
    assert np.abs(s["rpy"][:2]).max() < 0.2  # the bridge trips at 1 rad


def test_pd_tracks_a_pose(sim):
    target = np.zeros(NUM_JOINT)
    target[[3, 9]] = 0.3  # knees
    target[[18, 25]] = 0.8  # elbows
    s = run(sim, hold(target), 3.0)
    assert np.abs(s["q"] - target).max() < 0.05
    assert np.abs(s["dq"]).max() < 0.05


def test_disabled_motor_gives_no_torque(sim):
    cmd = hold(np.full(NUM_JOINT, 0.5))
    cmd.mode[:] = 0
    sim.step(cmd)
    assert not sim.state()["tau_est"].any()


def test_torque_is_clipped_to_the_motor(sim):
    cmd = hold(np.full(NUM_JOINT, 2.0))
    cmd.kp[:] = 1e4
    sim.step(cmd)
    assert np.all(np.abs(sim.state()["tau_est"]) <= sim.effort + 1e-9)
    assert abs(sim.state()["tau_est"][JOINT_NAMES.index("left_wrist_yaw_joint")]) == 5.0


def test_without_gantry_it_falls(sim):
    sim.gantry = False
    run(sim, Command(), 3.0)
    assert sim.data.qpos[2] < 0.5


def test_reset_puts_it_back_on_the_gantry(sim):
    sim.gantry = False
    run(sim, Command(), 1.0)
    sim.reset()
    assert sim.data.qpos[:3] == pytest.approx(sim.anchor)
    assert not sim.data.qpos[sim.qadr].any()


def test_static_accelerometer_reads_plus_g(sim):
    s = run(sim, hold(np.zeros(NUM_JOINT)), 2.0)  # limp legs keep the pelvis swinging
    assert s["accelerometer"][2] == pytest.approx(9.81, abs=0.3)


def test_rpy_from_quat_pitch():
    a = 0.4
    q = [np.cos(a / 2), 0.0, np.sin(a / 2), 0.0]
    assert rpy_from_quat(q) == pytest.approx([0.0, a, 0.0])
