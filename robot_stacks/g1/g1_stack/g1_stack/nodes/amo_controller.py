"""AMO through the bridge: /lowstate -> AMO (legs, waist) + the goal (arms, hands) -> /robot_cmd.

The port of psi0's `real/deploy/psi-inference_rtc.py` control tick, without its own rt/lowcmd:
gains from `robot_body.py`, hands from `robot_hand_unitree.py`, at AMO's 50 Hz. Arming is the
operator's step; the bridge drops /robot_cmd until then. Before the first goal the arms ramp
from the pose measured at the first tick to psi0's default arms (AMO balances best there).
No goal for GOAL_TIMEOUT: AMO stands, velocities 0, arms and hands at the last goal.
Without --walk the goal's vx, vy and vyaw are dropped: AMO stays in place and ignores the heading.
$G1_CONTROLLER_LOG: every tick's state, command and targets as JSONL (g1_stack.cmd_log).

Runs in psi0's venv (CUDA torch, pinocchio) with the ROS workspace sourced:

    source scripts/setup_unitree.sh
    PYTHONPATH=robot_stacks/g1/g1_stack:bridge/robot_bridge:$PYTHONPATH \\
        ~/github/psi0/.venv-psi/bin/python -m g1_stack.nodes.amo_controller
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
import threading
import time
from pathlib import Path

import numpy as np

from g1_stack import cmd_log
from g1_stack.amo import goal as goal_mod
from g1_stack.amo import policy as amo_policy
from g1_stack.amo.policy import DEFAULT_DOF_POS, DT, IN_PLACE_SPEED, NUM_MOTORS, STAND, Command
from g1_stack.amo.torque import MODES

NODE_NAME = "amo_bridge"
READY = "AMO -> /robot_cmd"  # the lab waits for this line
GOAL_TIMEOUT = 0.5  # s
STATE_TIMEOUT = 0.1  # s: older /lowstate sends nothing; the bridge's cmd_timeout holds
ARM_RAMP_S = 2.0  # measured arms -> default arms, before the first goal
ARMS = slice(15, 29)

# robot_body.py:191-212, as psi0 deployed them.
KP = np.array([
    150, 150, 150, 300, 80, 20,
    150, 150, 150, 300, 80, 20,
    400, 400, 400,
    50, 50, 50, 50, 30, 30, 30,
    50, 50, 50, 50, 30, 30, 30,
], dtype=np.float64)
KD = np.array([
    2, 2, 2, 4, 2, 1,
    2, 2, 2, 4, 2, 1,
    15, 15, 15,
    7.5, 7.5, 7.5, 7.5, 6, 6, 6,
    7.5, 7.5, 7.5, 7.5, 6, 6, 6,
], dtype=np.float64)
# Sent clipped to G1_config.yaml gate 5 (strong kp 350, kd 10): only the waist changes, 400/15 -> 350/10.
# The bridge would clip them anyway, with a warning on every message.
KP_SENT = np.minimum(KP, 350.0)
KD_SENT = np.minimum(KD, 10.0)
HAND_KP = 1.5  # robot_hand_unitree.py:58
HAND_KD = 0.2
HAND_SIDES = ("left", "right")


def default_amo_dir():
    return Path(os.environ.get("PSI0_ROOT", "~/github/psi0")).expanduser() / "real" / "teleop"


class Controller:
    """One `tick` per DT: AMO for legs and waist, the goal for arms and hands, then send."""

    def __init__(self, amo, gravity, client, hands=True, walk=False, clock=time.monotonic,
                 out=print):
        self.amo = amo
        self.gravity = gravity
        self.client = client
        self.hands = hands
        self.walk = walk
        self.clock = clock
        self.out = out
        self._command = STAND
        self._arm_q = None  # the last goal's arms
        self._arm_from = None  # (measured arms, clock) at the first tick
        self._hand_q = None
        self._goal_at = None  # clock() of the last goal taken
        self._last = None  # (targets, tau) of the last tick
        self._live = False
        self._walking_asked = False  # the goal's own walk command left the in-place band
        self.walk_goals = 0  # goals that asked to walk

    def take_goal(self, message):
        """A goal dict from the wire; a malformed one is reported and dropped."""
        try:
            command, arm_q, hand_q, _stamp = goal_mod.parse(message)
        except ValueError as exc:
            self.out(f"dropped goal: {exc}")
            return
        if not self._live:
            self.amo.reset_yaw()  # a new goal stream: its heading 0 is now
            self.out("goal stream: live")
        self._live = True
        self._command, self._arm_q, self._goal_at = command, arm_q, self.clock()
        if hand_q is not None:
            self._hand_q = hand_q
        asked = max(abs(command.vx), abs(command.vy), abs(command.vyaw)) >= IN_PLACE_SPEED
        self.walk_goals += asked
        if asked != self._walking_asked:
            self._walking_asked = asked
            what = "walk" if asked else "stand"
            done = "applied" if self.walk else "dropped (no --walk)"
            self.out(f"goal asks to {what}: vx {command.vx:+.2f} vy {command.vy:+.2f} "
                     f"vyaw {command.vyaw:+.3f} target_yaw {command.target_yaw:+.2f}; {done}")

    def command(self):
        """The command for this tick: the goal's, or standing still once it is stale."""
        if self._live and self.clock() - self._goal_at > GOAL_TIMEOUT:
            self._live = False
            self.out(f"goal stream: none for {GOAL_TIMEOUT} s; standing")
        if self._live and self.walk:
            return self._command
        c = self._command
        return Command(torso_roll=c.torso_roll, torso_pitch=c.torso_pitch, torso_yaw=c.torso_yaw,
                       torso_height=c.torso_height)

    def _start_arms(self):
        measured, at = self._arm_from
        a = min(1.0, (self.clock() - at) / ARM_RAMP_S)
        return (1 - a) * measured + a * DEFAULT_DOF_POS[ARMS]

    def tick(self, state):
        """`state`: (q29, dq29, quat wxyz, gyro). Returns the 29 targets sent."""
        q, dq, quat, gyro = state
        if self._arm_from is None:
            self._arm_from = (np.asarray(q, dtype=np.float64)[ARMS].copy(), self.clock())
        command = self.command()
        target = self.amo.step(q, dq, quat, gyro, command)
        target[ARMS] = self._arm_q if self._arm_q is not None else self._start_arms()
        tau = self.gravity(target)
        note = getattr(self.client, "note", None)
        if note is not None:  # cmd_log's client
            note(obs={"q": q, "dq": dq, "quat": quat, "gyro": gyro},
                 command={"navigate_cmd": [command.vx, command.vy, command.vyaw],
                          "base_height_command": command.torso_height,
                          "torso_rpy": [command.torso_roll, command.torso_pitch,
                                        command.torso_yaw],
                          "target_yaw": command.target_yaw})
        self.client.send_cmd(q_target_pos=target, target_tau=tau,
                             target_kp=KP_SENT, target_kd=KD_SENT, duration=DT, hold_position=False)
        self._last = (target, tau)
        if self.hands and self._hand_q is not None:
            for i, side in enumerate(HAND_SIDES):
                self.client.send_hand_cmd(side, q=self._hand_q[7 * i:7 * i + 7],
                                          kp=[HAND_KP] * 7, kd=[HAND_KD] * 7, duration=DT)
        return target

    def hold(self):
        """The last targets once more with hold_position: the bridge holds them, no cmd_timeout latch.
        False before the first tick."""
        if self._last is None:
            return False
        target, tau = self._last
        self.client.send_cmd(q_target_pos=target, target_tau=tau,
                             target_kp=KP_SENT, target_kd=KD_SENT, duration=DT, hold_position=True)
        return True


class LatestState:
    """The newest /lowstate, copied out under a lock."""

    def __init__(self, clock=time.monotonic):
        self._clock = clock
        self._lock = threading.Lock()
        self._state = None
        self._at = None

    def on_lowstate(self, msg):
        motors = msg.motor_state[:NUM_MOTORS]
        state = (np.array([m.q for m in motors], dtype=np.float64),
                 np.array([m.dq for m in motors], dtype=np.float64),
                 np.array(msg.imu_state.quaternion, dtype=np.float64),
                 np.array(msg.imu_state.gyroscope, dtype=np.float64))
        with self._lock:
            self._state, self._at = state, self._clock()

    def fresh(self):
        """The state if younger than STATE_TIMEOUT, else None."""
        with self._lock:
            if self._at is None or self._clock() - self._at > STATE_TIMEOUT:
                return None
            return self._state


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--amo-dir", type=Path, default=default_amo_dir(),
                   help="psi0 real/teleop: amo_jit.pt, adapter_jit.pt, adapter_norm_stats.pt "
                        "(default $PSI0_ROOT/real/teleop, PSI0_ROOT ~/github/psi0)")
    p.add_argument("--urdf", type=Path, default=None,
                   help="body URDF for the torque (default <amo-dir>/../assets/g1/g1_body29_hand14.urdf)")
    p.add_argument("--tau-ff", choices=MODES, default="arms",
                   help="gravity feedforward: all = psi0's deploy, arms = arms only, none")
    p.add_argument("--goal-host", default="127.0.0.1")
    p.add_argument("--goal-port", type=int, default=goal_mod.GOAL_PORT)
    p.add_argument("--device", default="cuda")
    p.add_argument("--no-hands", dest="hands", action="store_false", help="send no /hand_cmd")
    p.add_argument("--walk", action="store_true",
                   help="apply the goal's vx, vy, vyaw and heading; default: AMO stays in place")
    p.add_argument("--lowstate-topic", default="/lowstate")
    p.add_argument("--state-wait", type=float, default=10.0, help="s to wait for the first /lowstate")
    p.add_argument("--seconds", type=float, default=0.0,
                   help="stop after this long, counted from the first tick, then hold; 0: until Ctrl-C")
    args = p.parse_args(argv)
    if args.urdf is None:
        args.urdf = args.amo_dir.expanduser().parent / "assets" / "g1" / "g1_body29_hand14.urdf"
    return args


def run_loop(controller, state, goals, stop, clock=time.monotonic, sleep=time.sleep, out=print,
             seconds=0.0):
    """Ticks every DT until `stop` is set or `seconds` (0: no limit) ran out; a late tick is not made up."""
    next_at = clock()
    ends_at = next_at + seconds if seconds else None
    stale = False
    while not stop.is_set() and (ends_at is None or clock() < ends_at):
        message = goals.latest()
        if message is not None:
            controller.take_goal(message)
        current = state.fresh()
        if current is None:
            if not stale:
                out(f"/lowstate older than {STATE_TIMEOUT} s: sending nothing")
            stale = True
        else:
            if stale:
                out("/lowstate back")
            stale = False
            controller.tick(current)
        next_at += DT
        delay = next_at - clock()
        if delay > 0:
            sleep(delay)
        else:
            next_at = clock()


def main(argv=None):
    args = parse_args(argv)
    import rclpy
    from robot_bridge.cmd_client import RobotCmdClient
    from unitree_hg.msg import LowState

    from g1_stack.amo.torque import Gravity

    amo = amo_policy.load(args.amo_dir, args.device)
    gravity = Gravity(args.urdf, args.tau_ff)
    # rclpy's own SIGINT/SIGTERM handlers shut the context mid-tick; stop the loop instead, then hold.
    from rclpy.signals import SignalHandlerOptions

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node(NODE_NAME)
    state = LatestState()
    node.create_subscription(LowState, args.lowstate_topic, state.on_lowstate, 10)
    client = RobotCmdClient(node, num_dof=NUM_MOTORS, control_frequency=1.0 / DT,
                            interpolation_order=0.0)
    client = cmd_log.from_env(client, {
        "controller": "amo", "rate_hz": 1.0 / DT, "kp": KP_SENT, "kd": KD_SENT,
        "tau_ff": args.tau_ff, "walk": args.walk, "hands": args.hands})
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    goals = goal_mod.GoalSubscriber(args.goal_host, args.goal_port)
    stop = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: stop.set())
    controller = None
    try:
        deadline = time.monotonic() + args.state_wait
        while state.fresh() is None:
            if time.monotonic() > deadline:
                print(f"no {args.lowstate_topic} in {args.state_wait} s: is G1_bridge's robot up?",
                      file=sys.stderr)
                return 2
            time.sleep(0.05)
        controller = Controller(amo, gravity, client, hands=args.hands, walk=args.walk)
        print(f"{READY} (tau-ff {args.tau_ff}, hands {'on' if args.hands else 'off'}, "
              f"walk {'on' if args.walk else 'off'}, "
              f"goals tcp://{args.goal_host}:{args.goal_port})", flush=True)
        run_loop(controller, state, goals, stop, seconds=args.seconds)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        if controller is not None and controller.hold():
            print("held the last targets (hold_position): the bridge stays ARMED; Stop on the page",
                  flush=True)
        if controller is not None:
            print(f"goals that asked to walk: {controller.walk_goals}", flush=True)
        if hasattr(client, "close_log"):
            client.close_log()
        goals.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
