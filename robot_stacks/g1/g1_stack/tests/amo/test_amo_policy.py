"""The AMO port against psi0's own observation code. No ROS; the golden test needs torch and a psi0 clone."""

from __future__ import annotations

import ast
import os
from collections import deque
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from g1_stack.amo import policy
from g1_stack.amo.policy import STAND, Amo, Command

PSI0_MASTER = (Path(os.environ.get("PSI0_ROOT", "~/github/psi0")).expanduser()
               / "real" / "teleop" / "master_whole_body.py")


def fake_policy(obs, extra):
    """Deterministic, depends on both inputs, and goes past the +-40 clip."""
    return 60.0 * np.tanh(obs[:15] * 3.0 + extra[-15:])


def fake_adapter(x):
    return np.tanh(np.resize(x, 15))


def robot_state(rng):
    q = policy.DEFAULT_DOF_POS + rng.normal(0, 0.1, 29)
    dq = rng.normal(0, 1.0, 29)
    quat = rng.normal(0, 0.05, 4) + np.array([1.0, 0, 0, 0])
    gyro = rng.normal(0, 0.3, 3)
    return q, dq, quat / np.linalg.norm(quat), gyro


def test_observation_sizes_match_the_traced_policy():
    seen = {}

    def spy(obs, extra):
        seen["obs"], seen["extra"] = obs.shape, extra.shape
        return np.zeros(15)

    Amo(spy, fake_adapter).step(*robot_state(np.random.default_rng(0)))
    assert seen == {"obs": (1043,), "extra": (2325,)}
    assert (policy.N_OBS, policy.N_EXTRA) == (1043, 2325)


def test_step_moves_only_legs_and_waist_from_default():
    amo = Amo(lambda obs, extra: np.full(15, 100.0), fake_adapter)
    target = amo.step(*robot_state(np.random.default_rng(1)))
    np.testing.assert_allclose(target[:15], policy.DEFAULT_DOF_POS[:15] + 40.0 * 0.25)
    np.testing.assert_allclose(target[15:], policy.DEFAULT_DOF_POS[15:])


def test_quat_to_euler_identity_and_yaw():
    np.testing.assert_allclose(policy.quat_to_euler([1, 0, 0, 0]), [0, 0, 0], atol=1e-12)
    half = 0.3
    np.testing.assert_allclose(policy.quat_to_euler([np.cos(half), 0, 0, np.sin(half)]),
                               [0, 0, 2 * half], atol=1e-12)


def _psi0_functions():
    """psi0's `quatToEuler` and `RobotTaskmaster.get_ik_observation`, from source, unchanged."""
    tree = ast.parse(PSI0_MASTER.read_text())
    picked = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name in ("quatToEuler", "get_ik_observation"):
            picked.setdefault(node.name, node)
    module = ast.Module(body=[picked["quatToEuler"], picked["get_ik_observation"]], type_ignores=[])
    import torch

    namespace = {"np": np, "torch": torch}
    exec(compile(module, str(PSI0_MASTER), "exec"), namespace)  # noqa: S102 - the lab's own clone
    return namespace["quatToEuler"], namespace["get_ik_observation"], torch


@pytest.mark.skipif(not PSI0_MASTER.is_file(), reason=f"no psi0 clone at {PSI0_MASTER}")
def test_matches_psi0_get_ik_observation_over_a_run():
    pytest.importorskip("torch")
    quat_to_euler, get_ik_observation, torch = _psi0_functions()
    rng = np.random.default_rng(7)
    mean, std = rng.normal(0, 0.5, 12), rng.uniform(0.1, 1.0, 12)
    out_mean, out_std = rng.normal(0, 0.5, 15), rng.uniform(0.1, 0.6, 15)
    weights = torch.tensor(rng.normal(0, 0.3, (12, 15)), dtype=torch.float32)

    def adapter_net(t):
        return torch.tanh(t @ weights)

    def port_adapter(x):  # policy.load's wrapper around the same net
        t = torch.tensor(x).to("cpu", dtype=torch.float32).unsqueeze(0)
        t = (t - torch.tensor(mean, dtype=torch.float32)) / (torch.tensor(std, dtype=torch.float32) + 1e-8)
        out = adapter_net(t.view(1, -1)) * torch.tensor(out_std, dtype=torch.float32) \
            + torch.tensor(out_mean, dtype=torch.float32)
        return out.numpy().squeeze()

    # psi0's RobotTaskmaster state, as its __init__ sets it (master_whole_body.py:86-161).
    default = np.array([-0.1, 0.0, 0.0, 0.3, -0.2, 0.0, -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,
                        0.0, 0.0, 0.0, 0.5, 0.0, 0.2, 0.3, 0.0, 0.0, 0.0,
                        0.5, 0.0, -0.2, 0.3, 0.0, 0.0, 0.0])
    n_proprio = 3 + 2 + 2 + 23 * 3 + 2 + 15
    template = np.zeros(17)
    template[:8] = default[np.r_[15:19, 22:26]]
    template[14:17] = 0.75
    ref = SimpleNamespace(
        device="cpu", scales_ang_vel=0.25, scales_dof_vel=0.05, n_priv=3, _n_demo_dof=8,
        default_dof_pos=default, last_action=np.zeros(29), demo_obs_template=template,
        target_yaw=0.0, yaw_offset=0.0, dyaw=0.0, dt=0.02, _in_place_stand_flag=True,
        gait_cycle=np.array([0.25, 0.25]), gait_freq=1.3, control_dt=0.02,
        proprio_history_buf=deque([np.zeros(n_proprio)] * 10, maxlen=10),
        extra_history_buf=deque([np.zeros(n_proprio)] * 25, maxlen=25),
        adapter=adapter_net,
        input_mean=torch.tensor(mean, dtype=torch.float32), input_std=torch.tensor(std, dtype=torch.float32),
        output_mean=torch.tensor(out_mean, dtype=torch.float32),
        output_std=torch.tensor(out_std, dtype=torch.float32),
        vx=0.0, vy=0.0, vyaw=0.0)

    port_obs = []
    amo = Amo(lambda obs, extra: port_obs.append((obs, extra)) or fake_policy(obs, extra), port_adapter)
    # Stand, walk forward and turn, stand again: the in-place flag and the gait snap both flip.
    commands = ([STAND] * 20
                + [Command(torso_pitch=0.2, torso_height=0.7, vx=0.6, vyaw=0.3, target_yaw=0.4)] * 40
                + [Command(torso_roll=-0.1, torso_yaw=0.2, vy=-0.5)] * 20
                + [STAND] * 20)
    for i, cmd in enumerate(commands):
        q, dq, quat, gyro = robot_state(rng)
        port_target = amo.step(q, dq, quat, gyro, cmd)

        # psi-inference_rtc.py apply_action_from_buffer, then solve_whole_body_ik.
        ref.motorstate, ref.velstate, ref.ang_vel = q.copy(), dq.copy(), gyro.copy()
        ref.rpy = quat_to_euler(quat)
        if i == 0:
            ref.yaw_offset = ref.rpy[2]
        ref.torso_roll, ref.torso_pitch, ref.torso_yaw = cmd.torso_roll, cmd.torso_pitch, cmd.torso_yaw
        ref.torso_height = cmd.torso_height
        ref.vx, ref.vy, ref.vyaw, ref.target_yaw = cmd.vx, cmd.vy, cmd.vyaw, cmd.target_yaw
        obs, extra = get_ik_observation(ref, record=False)
        extra = np.array(extra).flatten()
        raw = np.clip(fake_policy(obs, extra), -40.0, 40.0)
        ref_target = np.concatenate([raw * 0.25, np.zeros(14)]) + default
        ref.last_action = np.concatenate([raw.copy(), (q - default)[15:] / 0.25])

        np.testing.assert_allclose(port_obs[-1][0], obs, rtol=1e-6, atol=1e-6, err_msg=f"obs, step {i}")
        np.testing.assert_allclose(port_obs[-1][1], extra, rtol=1e-6, atol=1e-6, err_msg=f"extra, step {i}")
        np.testing.assert_allclose(port_target, ref_target, rtol=1e-6, atol=1e-6, err_msg=f"target, step {i}")


def test_reset_yaw_takes_the_next_measured_yaw_as_heading_zero():
    seen = []
    amo = Amo(lambda obs, extra: seen.append(obs[5:7].copy()) or np.zeros(15), fake_adapter)
    walk = Command(vx=0.6)
    q, dq = policy.DEFAULT_DOF_POS.copy(), np.zeros(29)

    def yawed(yaw):
        return [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]

    amo.step(q, dq, yawed(0.0), np.zeros(3), walk)  # still in place: dyaw 0, flag flips
    amo.step(q, dq, yawed(0.5), np.zeros(3), walk)
    np.testing.assert_allclose(seen[-1], [np.sin(0.5), np.cos(0.5)], atol=1e-9)
    amo.reset_yaw()
    amo.step(q, dq, yawed(0.5), np.zeros(3), walk)
    np.testing.assert_allclose(seen[-1], [0.0, 1.0], atol=1e-9)
