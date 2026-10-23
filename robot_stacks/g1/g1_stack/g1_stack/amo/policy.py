"""AMO, the lower-body policy psi0 was trained with: observation, history, gait, action.

Ported from psi0 `real/teleop/master_whole_body.py` (`get_ik_observation`, `maintain_standing`)
and `robot_control/robot_body_ik.py` (`solve_whole_body_ik`), psi0 a32e57a. The policy and the
adapter are injected callables, so this module needs numpy only; `load` brings in torch.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np

NUM_MOTORS = 29
NUM_LOWER = 15  # what AMO outputs: legs 12 + waist 3

# master_whole_body.py:97-103, motor order.
DEFAULT_DOF_POS = np.array([
    -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,
    -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,
    0.0, 0.0, 0.0,
    0.5, 0.0, 0.2, 0.3, 0.0, 0.0, 0.0,
    0.5, 0.0, -0.2, 0.3, 0.0, 0.0, 0.0,
])

OBS_IDX = np.r_[0:19, 22:26]  # legs, waist, shoulders and elbows: no wrists
ZERO_VEL_IDX = [4, 5, 10, 11, 13, 14]  # in OBS_IDX: ankles, waist roll and pitch
NUM_DEMO_DOF = 8  # the 4+4 shoulder and elbow joints of OBS_IDX[15:]

ACTION_SCALE = 0.25
ACTION_CLIP = 40.0
SCALE_ANG_VEL = 0.25
SCALE_DOF_VEL = 0.05
N_PROPRIO = 3 + 2 + 2 + 23 * 3 + 2 + 15
N_PRIV = 3
HISTORY_LEN = 10
EXTRA_HISTORY_LEN = 25
N_OBS = N_PROPRIO + (NUM_DEMO_DOF + 9) + N_PRIV + HISTORY_LEN * N_PROPRIO  # 1043
N_EXTRA = EXTRA_HISTORY_LEN * N_PROPRIO  # 2325

DT = 0.02  # AMO's step; run it at 50 Hz or the gait phase runs fast
GAIT_FREQ = 1.3
STAND_HEIGHT = 0.75
IN_PLACE_SPEED = 0.1  # |vx|, |vy|, |vyaw| below this: stand in place

ADAPTER_EPS = 1e-8


@dataclass(frozen=True)
class Command:
    """What AMO tracks: torso orientation and height, base velocity, heading."""

    torso_roll: float = 0.0
    torso_pitch: float = 0.0
    torso_yaw: float = 0.0
    torso_height: float = STAND_HEIGHT
    vx: float = 0.0
    vy: float = 0.0
    vyaw: float = 0.0
    target_yaw: float = 0.0


STAND = Command()  # maintain_standing's command


def quat_to_euler(quat):
    """(roll, pitch, yaw) from a (w, x, y, z) quaternion; master_whole_body.py:34."""
    qw, qx, qy, qz = (float(v) for v in quat)
    roll = np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    sinp = 2 * (qw * qy - qz * qx)
    pitch = np.copysign(np.pi / 2, sinp) if abs(sinp) >= 1 else np.arcsin(sinp)
    yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return np.array([roll, pitch, yaw])


def wrap(angle):
    return np.remainder(angle + np.pi, 2 * np.pi) - np.pi


class Amo:
    """One AMO step per `step` call, at DT. `policy(obs, extra) -> (15,)`, `adapter(x12) -> (15,)`."""

    def __init__(self, policy, adapter):
        self._policy = policy
        self._adapter = adapter
        self.reset()

    def reset(self):
        self._hist = deque([np.zeros(N_PROPRIO)] * HISTORY_LEN, maxlen=HISTORY_LEN)
        self._extra = deque([np.zeros(N_PROPRIO)] * EXTRA_HISTORY_LEN, maxlen=EXTRA_HISTORY_LEN)
        self._last_action = np.zeros(NUM_MOTORS)
        self._gait_cycle = np.array([0.25, 0.25])
        self._in_place = True
        self._yaw_offset = None

    def reset_yaw(self):
        """The next step takes the measured yaw as heading 0 (psi-inference_rtc.py:268)."""
        self._yaw_offset = None

    def observe(self, q, dq, quat, gyro, cmd: Command):
        """(obs (1043,), extra_hist (2325,)) and the step's bookkeeping; get_ik_observation(record=False)."""
        q = np.asarray(q, dtype=np.float64)
        rpy = quat_to_euler(quat)
        if self._yaw_offset is None:
            self._yaw_offset = rpy[2]
        dyaw = 0.0 if self._in_place else wrap(rpy[2] - self._yaw_offset - cmd.target_yaw)

        dof_pos = q[OBS_IDX]
        dof_vel = np.asarray(dq, dtype=np.float64)[OBS_IDX]
        dof_vel[ZERO_VEL_IDX] = 0.0
        arm_pos = dof_pos[15:]

        adapter_in = np.concatenate([
            [cmd.torso_height, cmd.torso_yaw, cmd.torso_pitch, cmd.torso_roll], arm_pos])
        adapter_out = np.asarray(self._adapter(adapter_in), dtype=np.float64).reshape(NUM_LOWER)

        prop = np.concatenate([
            np.asarray(gyro, dtype=np.float64) * SCALE_ANG_VEL,
            rpy[:2],
            (np.sin(dyaw), np.cos(dyaw)),
            dof_pos - DEFAULT_DOF_POS[OBS_IDX],
            dof_vel * SCALE_DOF_VEL,
            self._last_action[OBS_IDX],
            np.sin(self._gait_cycle * 2 * np.pi),
            adapter_out,
        ])
        hist = np.array(self._hist).flatten()  # the 10 steps before this one

        # The flag is set here, after dyaw used the previous one (master_whole_body.py:373).
        self._in_place = (abs(cmd.vx) < IN_PLACE_SPEED and abs(cmd.vy) < IN_PLACE_SPEED
                          and abs(cmd.vyaw) < IN_PLACE_SPEED)
        demo = np.zeros(NUM_DEMO_DOF + 9)
        demo[:NUM_DEMO_DOF] = arm_pos
        demo[NUM_DEMO_DOF] = cmd.vx
        demo[NUM_DEMO_DOF + 1] = cmd.vy  # [+2] stays 0: vyaw reaches AMO through dyaw only
        demo[NUM_DEMO_DOF + 3] = cmd.torso_yaw
        demo[NUM_DEMO_DOF + 4] = cmd.torso_pitch
        demo[NUM_DEMO_DOF + 5] = cmd.torso_roll
        demo[NUM_DEMO_DOF + 6:NUM_DEMO_DOF + 9] = cmd.torso_height

        self._hist.append(prop)
        self._extra.append(prop)
        obs = np.concatenate([prop, demo, np.zeros(N_PRIV), hist])
        extra = np.array(self._extra).flatten()  # the 25 steps up to and with this one

        self._gait_cycle = np.remainder(self._gait_cycle + DT * GAIT_FREQ, 1.0)
        near = np.abs(self._gait_cycle - 0.25) < 0.05
        if self._in_place and near.any():
            self._gait_cycle = np.array([0.25, 0.25])
        if not self._in_place and near.all():
            self._gait_cycle = np.array([0.25, 0.75])
        return obs, extra

    def step(self, q, dq, quat, gyro, cmd: Command = STAND):
        """The 29 joint targets: AMO's legs and waist, the default arms. Callers overwrite the arms."""
        obs, extra = self.observe(q, dq, quat, gyro, cmd)
        raw = np.clip(np.asarray(self._policy(obs, extra), dtype=np.float64).reshape(NUM_LOWER),
                      -ACTION_CLIP, ACTION_CLIP)
        target = DEFAULT_DOF_POS.copy()
        target[:NUM_LOWER] += raw * ACTION_SCALE
        self._last_action = np.concatenate([
            raw, (np.asarray(q, dtype=np.float64) - DEFAULT_DOF_POS)[NUM_LOWER:] / ACTION_SCALE])
        return target


def load(amo_dir, device="cuda"):
    """An Amo running psi0's `amo_jit.pt` and `adapter_jit.pt` from `amo_dir` (psi0 real/teleop)."""
    import torch

    amo_dir = Path(amo_dir).expanduser()
    # The traced AMO graph creates tensors on cuda:0; it does not run on the CPU.
    net = torch.jit.load(str(amo_dir / "amo_jit.pt"), map_location=device)
    adapter = torch.jit.load(str(amo_dir / "adapter_jit.pt"), map_location=device)
    adapter.eval()
    stats = torch.load(str(amo_dir / "adapter_norm_stats.pt"), weights_only=False)
    mean, std, out_mean, out_std = (
        torch.tensor(stats[k], device=device, dtype=torch.float32)
        for k in ("input_mean", "input_std", "output_mean", "output_std"))

    def run_adapter(x):
        with torch.no_grad():
            t = torch.tensor(x).to(device, dtype=torch.float32).unsqueeze(0)
            out = adapter(((t - mean) / (std + ADAPTER_EPS)).view(1, -1)) * out_std + out_mean
        return out.cpu().numpy().squeeze()

    def run_policy(obs, extra):
        with torch.no_grad():
            o = torch.from_numpy(obs).float().unsqueeze(0).to(device)
            e = torch.tensor(extra, dtype=torch.float).view(1, -1).to(device)
            return net(o, e).cpu().numpy().squeeze()

    return Amo(run_policy, run_adapter)
