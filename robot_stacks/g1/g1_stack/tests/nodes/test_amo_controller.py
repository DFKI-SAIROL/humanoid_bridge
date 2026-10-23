"""The AMO controller's tick and loop. No ROS, no torch: AMO, client and clock are fakes."""

from __future__ import annotations

import threading
from types import SimpleNamespace

import numpy as np

from g1_stack.amo import goal
from g1_stack.amo.policy import DEFAULT_DOF_POS, DT, STAND, Command
from g1_stack.nodes import amo_controller as ac


class FakeAmo:
    def __init__(self):
        self.commands = []
        self.yaw_resets = 0

    def reset_yaw(self):
        self.yaw_resets += 1

    def step(self, q, dq, quat, gyro, cmd):
        self.commands.append(cmd)
        target = DEFAULT_DOF_POS.copy()
        target[:15] += 0.01
        return target


class FakeClient:
    def __init__(self):
        self.cmds = []
        self.hands = []

    def send_cmd(self, **kwargs):
        self.cmds.append(kwargs)

    def send_hand_cmd(self, side, **kwargs):
        self.hands.append((side, kwargs))


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


STATE = (DEFAULT_DOF_POS.copy(), np.zeros(29), np.array([1.0, 0, 0, 0]), np.zeros(3))


def make(hands=True, walk=False):
    clock, lines = Clock(), []
    ctl = ac.Controller(FakeAmo(), lambda q: np.full(29, 0.5), FakeClient(), hands=hands,
                        walk=walk, clock=clock, out=lines.append)
    return ctl, clock, lines


def a_goal(**kwargs):
    return goal.make_goal(np.full(14, 0.3), hand_q=np.full(14, 0.2), **kwargs)


def test_gains_are_psi0_robot_body_tables():
    leg = [150, 150, 150, 300, 80, 20]
    arm = [50, 50, 50, 50, 30, 30, 30]
    assert ac.KP.tolist() == leg + leg + [400] * 3 + arm + arm
    leg_d = [2, 2, 2, 4, 2, 1]
    arm_d = [7.5, 7.5, 7.5, 7.5, 6, 6, 6]
    assert ac.KD.tolist() == leg_d + leg_d + [15] * 3 + arm_d + arm_d
    assert (ac.HAND_KP, ac.HAND_KD) == (1.5, 0.2)


def test_sent_gains_are_the_tables_clipped_to_the_bridge_envelope():
    changed = np.flatnonzero((ac.KP_SENT != ac.KP) | (ac.KD_SENT != ac.KD)).tolist()
    assert changed == [12, 13, 14], "only the waist exceeds G1_config.yaml gate 5"
    assert ac.KP_SENT[12:15].tolist() == [350.0] * 3 and ac.KD_SENT[12:15].tolist() == [10.0] * 3


def test_before_any_goal_amo_stands_and_the_arms_ramp_from_measured_to_default():
    ctl, clock, _ = make()
    q = DEFAULT_DOF_POS.copy()
    q[15:] = 0.7
    target = ctl.tick((q,) + STATE[1:])
    assert ctl.amo.commands == [STAND]
    np.testing.assert_allclose(target[15:], np.full(14, 0.7))
    clock.t += ac.ARM_RAMP_S / 2
    target = ctl.tick(STATE)
    np.testing.assert_allclose(target[15:], (np.full(14, 0.7) + DEFAULT_DOF_POS[15:]) / 2)
    clock.t += ac.ARM_RAMP_S
    target = ctl.tick(STATE)
    np.testing.assert_allclose(target[15:], DEFAULT_DOF_POS[15:])
    sent = ctl.client.cmds[-1]
    np.testing.assert_allclose(sent["q_target_pos"], target)
    np.testing.assert_allclose(sent["target_tau"], np.full(29, 0.5))
    assert sent["target_kp"] is ac.KP_SENT and sent["target_kd"] is ac.KD_SENT
    assert (sent["duration"], sent["hold_position"]) == (DT, False)
    assert ctl.client.hands == []


def test_goal_drives_arms_hands_and_command():
    ctl, _, lines = make(walk=True)
    ctl.take_goal(a_goal(torso_rpy=(0.1, 0.2, 0.3), torso_height=0.7, vx=0.6, target_yaw=0.4))
    target = ctl.tick(STATE)
    assert ctl.amo.commands[-1] == Command(0.1, 0.2, 0.3, 0.7, vx=0.6, target_yaw=0.4)
    np.testing.assert_allclose(target[15:], np.full(14, 0.3))
    np.testing.assert_allclose(target[:15], DEFAULT_DOF_POS[:15] + 0.01)
    assert [side for side, _ in ctl.client.hands] == ["left", "right"]
    side, kwargs = ctl.client.hands[0]
    np.testing.assert_allclose(kwargs["q"], np.full(7, 0.2))
    assert (kwargs["kp"], kwargs["kd"]) == ([1.5] * 7, [0.2] * 7)
    assert ctl.amo.yaw_resets == 1
    assert lines == ["goal stream: live",
                     "goal asks to walk: vx +0.60 vy +0.00 vyaw +0.000 target_yaw +0.40; applied"]


def test_without_walk_amo_stays_in_place_and_logs_what_it_dropped():
    ctl, _, lines = make()
    ctl.take_goal(a_goal(torso_rpy=(0.1, 0.2, 0.3), torso_height=0.7, vyaw=0.15, target_yaw=-1.05))
    ctl.tick(STATE)
    assert ctl.amo.commands[-1] == Command(0.1, 0.2, 0.3, 0.7), "torso kept, walk and heading dropped"
    ctl.take_goal(a_goal(vyaw=0.15))
    ctl.take_goal(a_goal())
    assert lines[1:] == [
        "goal asks to walk: vx +0.00 vy +0.00 vyaw +0.150 target_yaw -1.05; dropped (no --walk)",
        "goal asks to stand: vx +0.00 vy +0.00 vyaw +0.000 target_yaw +0.00; dropped (no --walk)"]
    assert ctl.walk_goals == 2


def test_no_hands_flag_sends_no_hand_cmd():
    ctl, _, _ = make(hands=False)
    ctl.take_goal(a_goal())
    ctl.tick(STATE)
    assert ctl.client.hands == []


def test_stale_goal_stands_still_keeps_pose_and_a_new_stream_resets_yaw():
    ctl, clock, lines = make()
    ctl.take_goal(a_goal(torso_rpy=(0.1, 0.2, 0.3), torso_height=0.7, vx=0.6, vy=0.5, vyaw=0.3,
                         target_yaw=0.4))
    ctl.tick(STATE)
    clock.t += ac.GOAL_TIMEOUT + 0.01
    target = ctl.tick(STATE)
    assert ctl.amo.commands[-1] == Command(0.1, 0.2, 0.3, 0.7)
    np.testing.assert_allclose(target[15:], np.full(14, 0.3))
    assert lines[-1] == f"goal stream: none for {ac.GOAL_TIMEOUT} s; standing"
    ctl.take_goal(a_goal())
    assert ctl.amo.yaw_resets == 2


def test_malformed_goal_is_dropped_and_reported():
    ctl, _, lines = make()
    ctl.take_goal({**a_goal(), "arm_q": [0.0]})
    ctl.tick(STATE)
    assert ctl.amo.commands == [STAND]
    assert lines == ["dropped goal: goal arm_q: expected 14 values"]


def test_latest_state_copies_lowstate_and_goes_stale():
    clock = Clock()
    state = ac.LatestState(clock=clock)
    assert state.fresh() is None
    motors = [SimpleNamespace(q=0.1 * i, dq=-0.1 * i) for i in range(35)]
    imu = SimpleNamespace(quaternion=[1.0, 0.0, 0.0, 0.0], gyroscope=[0.1, 0.2, 0.3])
    state.on_lowstate(SimpleNamespace(motor_state=motors, imu_state=imu))
    q, dq, quat, gyro = state.fresh()
    assert q.shape == (29,) and q[28] == 0.1 * 28 and dq[1] == -0.1
    np.testing.assert_allclose(gyro, [0.1, 0.2, 0.3])
    clock.t += ac.STATE_TIMEOUT + 0.01
    assert state.fresh() is None


def test_run_loop_skips_ticks_without_fresh_state():
    ctl, _, _ = make()
    lines, stop = [], threading.Event()
    states = iter([STATE, None, None, STATE])
    goals = SimpleNamespace(latest=lambda: None)
    feed = SimpleNamespace(fresh=lambda: next(states))
    sleeps = []

    def sleep(delay):
        sleeps.append(delay)
        if len(sleeps) == 4:
            stop.set()

    t = [0.0]
    ac.run_loop(ctl, feed, goals, stop, clock=lambda: t[0], sleep=sleep, out=lines.append)
    assert len(ctl.client.cmds) == 2
    assert lines == [f"/lowstate older than {ac.STATE_TIMEOUT} s: sending nothing", "/lowstate back"]
    assert sleeps == [DT, 2 * DT, 3 * DT, 4 * DT]


def test_hold_resends_the_last_targets_with_hold_position():
    ctl, _, _ = make()
    assert ctl.hold() is False, "nothing to hold before the first tick"
    target = ctl.tick(STATE)
    assert ctl.hold() is True
    last = ctl.client.cmds[-1]
    np.testing.assert_allclose(last["q_target_pos"], target)
    assert last["hold_position"] is True and ctl.client.cmds[-2]["hold_position"] is False


def test_run_loop_stops_after_seconds():
    ctl, _, _ = make()
    t = [0.0]

    def sleep(delay):
        t[0] += delay

    ac.run_loop(ctl, SimpleNamespace(fresh=lambda: STATE), SimpleNamespace(latest=lambda: None),
                threading.Event(), clock=lambda: t[0], sleep=sleep, out=lambda _: None, seconds=0.1)
    assert len(ctl.client.cmds) == 5


def test_urdf_defaults_beside_the_amo_dir(tmp_path):
    args = ac.parse_args(["--amo-dir", str(tmp_path / "real" / "teleop")])
    assert args.urdf == tmp_path / "real" / "assets" / "g1" / "g1_body29_hand14.urdf"
    assert (args.tau_ff, args.hands, args.goal_port, args.walk) == ("arms", True, 6001, False)
