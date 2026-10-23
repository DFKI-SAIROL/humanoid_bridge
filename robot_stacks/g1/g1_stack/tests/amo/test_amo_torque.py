"""The gravity feedforward against psi0's own pinocchio models. Needs pinocchio and a psi0 clone."""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np
import pytest

from g1_stack.amo.policy import DEFAULT_DOF_POS
from g1_stack.amo.torque import Gravity

URDF = (Path(os.environ.get("PSI0_ROOT", "~/github/psi0")).expanduser()
        / "real" / "assets" / "g1" / "g1_body29_hand14.urdf")

# G1_config.yaml joint_names, motor order.
MOTOR_ORDER = [
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee", "left_ankle_pitch", "left_ankle_roll",
    "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee", "right_ankle_pitch",
    "right_ankle_roll", "waist_yaw", "waist_roll", "waist_pitch",
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow",
    "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw",
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw", "right_elbow",
    "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw",
]


def test_none_needs_no_pinocchio_and_sends_zeros():
    np.testing.assert_array_equal(Gravity("/nonexistent.urdf", "none")(DEFAULT_DOF_POS), np.zeros(29))


def test_unknown_mode_refused():
    with pytest.raises(ValueError, match="one of none, arms, all"):
        Gravity(URDF, "legs")


needs_model = pytest.mark.skipif(not URDF.is_file(), reason=f"no psi0 URDF at {URDF}")


@needs_model
def test_urdf_joint_order_is_the_motor_order():
    pytest.importorskip("pinocchio")
    assert Gravity(URDF, "all").joint_names == [f"{n}_joint" for n in MOTOR_ORDER]


@needs_model
def test_modes_match_psi0_deploy():
    pin = pytest.importorskip("pinocchio")
    robot = pin.RobotWrapper.BuildFromURDF(str(URDF), str(URDF.parent))
    zero = np.zeros(robot.model.nq)
    # robot_body_ik.py:123 and compute_tau.py GetTauer, built the way psi0 builds them.
    hands = [n for n in robot.model.names[1:] if "_hand_" in n]
    whole = robot.buildReducedRobot(list_of_joints_to_lock=hands, reference_configuration=zero)
    arm_lock = [f"{n}_joint" for n in MOTOR_ORDER[:15]] + hands
    arms = robot.buildReducedRobot(list_of_joints_to_lock=arm_lock, reference_configuration=zero)

    q = DEFAULT_DOF_POS + np.random.default_rng(3).normal(0, 0.2, 29)
    whole_tau = pin.rnea(whole.model, whole.data, q, np.zeros(29), np.zeros(29))
    arm_tau = pin.rnea(arms.model, arms.data, q[15:], np.zeros(14), np.zeros(14))

    all_ff = Gravity(URDF, "all")(q)
    np.testing.assert_allclose(all_ff[:15], whole_tau[:15], atol=1e-9)
    np.testing.assert_allclose(all_ff[15:], arm_tau, atol=1e-9)
    arms_ff = Gravity(URDF, "arms")(q)
    np.testing.assert_array_equal(arms_ff[:15], np.zeros(15))
    np.testing.assert_allclose(arms_ff[15:], arm_tau, atol=1e-9)
    forward, back = np.zeros(29), np.zeros(29)
    forward[[15, 22]], back[[15, 22]] = -1.57, 1.57
    assert Gravity(URDF, "arms")(forward)[15] < -3.0 < 3.0 < Gravity(URDF, "arms")(back)[15], \
        "a horizontal arm needs about 5 Nm at the shoulder, opposite signs front and back"
