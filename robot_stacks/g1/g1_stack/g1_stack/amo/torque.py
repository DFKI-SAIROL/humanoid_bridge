"""Gravity torque feedforward as psi0's deploy computes it: pinocchio RNEA, fixed pelvis, zero velocity.

`all` is psi-inference_rtc.py:356-372: whole-body RNEA for legs and waist, `GetTauer` for the arms.
`arms` keeps only the arm part; `none` sends no feedforward.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

NUM_MOTORS = 29
LOWER = slice(0, 15)
ARMS = slice(15, 29)
MODES = ("none", "arms", "all")


def _is_hand(name):
    return "_hand_" in name


class Gravity:
    """`gravity(q29) -> tau29` for the targets about to be sent."""

    def __init__(self, urdf, mode="arms"):
        if mode not in MODES:
            raise ValueError(f"tau feedforward {mode!r}: one of {', '.join(MODES)}")
        self.mode = mode
        self._pin = None
        if mode == "none":
            return
        import pinocchio as pin

        urdf = Path(urdf).expanduser()
        robot = pin.RobotWrapper.BuildFromURDF(str(urdf), str(urdf.parent))
        zero = np.zeros(robot.model.nq)
        # robot_body_ik.py:123: the 29 body joints, hands locked at 0.
        whole = robot.buildReducedRobot(
            list_of_joints_to_lock=[n for n in robot.model.names[1:] if _is_hand(n)],
            reference_configuration=zero)
        # compute_tau.py GetTauer: the 14 arm joints, legs, waist and hands locked at 0.
        arm_names = list(whole.model.names[1:])[ARMS]
        arms = robot.buildReducedRobot(
            list_of_joints_to_lock=[n for n in robot.model.names[1:] if n not in arm_names],
            reference_configuration=zero)
        if whole.model.nq != NUM_MOTORS or arms.model.nq != ARMS.stop - ARMS.start:
            raise ValueError(f"{urdf}: {whole.model.nq} body and {arms.model.nq} arm joints, "
                             f"not {NUM_MOTORS} and 14")
        self._pin = pin
        self._whole = (whole.model, whole.model.createData())
        self._arms = (arms.model, arms.model.createData())
        self.joint_names = list(whole.model.names[1:])

    def _rnea(self, model_data, q):
        model, data = model_data
        q = np.asarray(q, dtype=np.float64)
        return np.asarray(self._pin.rnea(model, data, q, np.zeros(model.nv), np.zeros(model.nv)))

    def __call__(self, q):
        tau = np.zeros(NUM_MOTORS)
        if self.mode == "none":
            return tau
        q = np.asarray(q, dtype=np.float64)
        if self.mode == "all":
            tau[LOWER] = self._rnea(self._whole, q)[LOWER]
        tau[ARMS] = self._rnea(self._arms, q[ARMS])
        return tau
