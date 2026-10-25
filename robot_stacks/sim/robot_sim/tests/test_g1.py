"""RobotSim on the G1, no ROS. Needs mujoco and the mjlab g1.xml (fetch_assets.sh); skips without."""

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

from robot_sim.physics import Command, HandCommand, RobotSim, rpy_from_quat  # noqa: E402
from robot_sim.robots.g1 import (  # noqa: E402
    DEFAULT_MJCF,
    HAND_JOINTS,
    HAND_SIDES,
    JOINT_NAMES,
    NUM_HAND_JOINT,
    NUM_JOINT,
    SPEC,
)

pytestmark = pytest.mark.skipif(not DEFAULT_MJCF.is_file(), reason=f"no {DEFAULT_MJCF}")

KP = np.array([100, 100, 100, 150, 40, 40] * 2 + [300] * 3 + [100, 100, 50, 50, 20, 20, 20] * 2)
KD = np.array([2, 2, 2, 4, 2, 2] * 2 + [5] * 3 + [2, 2, 2, 2, 1, 1, 1] * 2)


@pytest.fixture
def sim():
    return RobotSim(SPEC)


def hold(target):
    cmd = Command(NUM_JOINT)
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
    s = run(sim, Command(NUM_JOINT), 2.0)
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
    run(sim, Command(NUM_JOINT), 3.0)
    assert sim.data.qpos[2] < 0.5


def test_reset_puts_it_back_on_the_gantry(sim):
    sim.gantry = False
    run(sim, Command(NUM_JOINT), 1.0)
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


# unitree's G1 with Dex3 hands, beside mjlab's (fetch_assets.sh --model unitree).
DEX3_MJCF = DEFAULT_MJCF.parent.parent / "g1_description" / "g1_29dof_with_hand.xml"
dex3_only = pytest.mark.skipif(not DEX3_MJCF.is_file(), reason=f"no {DEX3_MJCF}")


@pytest.fixture
def dex3():
    return RobotSim(SPEC, DEX3_MJCF)


def test_mjlab_has_no_hands(sim):
    assert sim.hands is False
    assert sim.hand_qadr == {}


@dex3_only
def test_dex3_model_has_both_hands_in_the_bridge_order(dex3):
    assert dex3.hands is True
    for side in HAND_SIDES:
        names = [dex3.model.joint(dex3.model.jnt_qposadr.tolist().index(a)).name for a in dex3.hand_qadr[side]]
        assert names == list(HAND_JOINTS[side])
    assert dex3.hand_effort["left"][0] == pytest.approx(2.45), "thumb_0 from the MJCF"
    assert dex3.dt == pytest.approx(0.002)


@dex3_only
def test_dex3_fingers_track_a_hand_command_and_the_body_still_holds(dex3):
    hand = HandCommand(NUM_HAND_JOINT)
    hand.q[:] = [0.3, 0.3, 0.6, -0.5, -0.5, -0.5, -0.5]
    hand.kp[:], hand.kd[:] = 2.0, 0.1
    body = hold(np.zeros(NUM_JOINT))
    for _ in range(int(2.0 / dex3.dt)):
        dex3.step(body, {"left": hand})
    left, right = dex3.hand_state("left"), dex3.hand_state("right")
    assert np.abs(left["q"] - hand.q).max() < 0.05
    assert not right["tau_est"].any(), "no command: the right hand is slack"
    assert np.abs(dex3.state()["q"]).max() < 0.1


@dex3_only
def test_dex3_finger_torque_is_clipped_to_the_mjcf_effort(dex3):
    hand = HandCommand(NUM_HAND_JOINT)
    hand.q[:], hand.kp[:] = 1.0, 1e3
    dex3.step(Command(NUM_JOINT), {"left": hand, "right": hand})
    for side in HAND_SIDES:
        assert np.all(np.abs(dex3.hand_state(side)["tau_est"]) <= dex3.hand_effort[side] + 1e-9)


def test_the_robots_registry_loads_the_g1():
    from robot_sim import robots

    assert robots.load("g1").SPEC is SPEC
    with pytest.raises(ValueError, match="expected one of"):
        robots.load("h1")


def test_unitree_buttons_land_on_their_key_bits():
    from robot_sim.robots import unitree

    raw = [0] * 40
    unitree.set_keys(raw, unitree.key_mask(["R1", "r2"]))
    assert raw[2:4] == [0b10001, 0], "R1 is bit 0, R2 bit 4, little-endian"
    unitree.set_remote(raw, False)
    assert raw[:2] == [0, 0]
    with pytest.raises(ValueError, match="button 'RB'"):
        unitree.key_mask(["RB"])
