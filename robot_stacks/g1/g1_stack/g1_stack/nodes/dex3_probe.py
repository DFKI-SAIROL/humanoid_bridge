"""Read a Dex3-1 hand off DDS, and optionally command it.

Two jobs, one file, because the command half is only ever safe to run right
after the read half has shown the hand alive:

- `read` (default): print `rt/dex3/{side}/state` live. Read-only.
- `command`: publish `rt/dex3/{side}/cmd`. **This moves fingers.**

Both talk raw `unitree_sdk2py` DDS, not through the bridge or GEAR. The G1
bridge's hand path uses the same topics (`docs/hands_path.md`) and cannot see
this probe's publisher.

Lab machine, over the wire, from a venv that has `unitree_sdk2py` (SIMPLE's):

    dex3_probe --iface enp4s0

Jetson, where the SDK is system-wide and the interface auto-detects:

    dex3_probe

The `moved` column answers the open "Dex3 left/right read order" question in
`PROJECT_STATUS.md`:
bend one finger by hand and watch which side and which motor index answers.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

from g1_stack import robot
from g1_stack.joints import dds as joint_sources

SIDES = ("left", "right")

NUM_MOTORS = robot.NUM_HAND_JOINTS_PER_HAND

# Motor index -> joint name and limits. Order is the DDS message order, from
# psi0 `real/teleop/robot_control/hand_retargeting.py:40-46`, which cites
# Unitree's "Sort by message structure". Limits from the URDFs in
# `psi0/real/assets/unitree_hand/`. Both hands share the index order; the
# ranges are mirrored, so the left hand closes negative and the right closes
# positive. NOT the order in `unitree_dex3.yml` -> that is the retargeting
# library's, which is why upstream remaps.
#
# Still unconfirmed against our hardware; `read` mode is what confirms it.
# Open question "Dex3 left/right read order" in PROJECT_STATUS.md.
LIMITS = {
    "left": [
        ("thumb_0", -1.0472, 1.0472),
        ("thumb_1", -0.7243, 0.9200),
        ("thumb_2", 0.0000, 1.7453),
        ("middle_0", -1.5708, 0.0000),
        ("middle_1", -1.7453, 0.0000),
        ("index_0", -1.5708, 0.0000),
        ("index_1", -1.7453, 0.0000),
    ],
    "right": [
        ("thumb_0", -1.0472, 1.0472),
        ("thumb_1", -0.9200, 0.7243),
        ("thumb_2", -1.7453, 0.0000),
        ("middle_0", 0.0000, 1.5708),
        ("middle_1", 0.0000, 1.7453),
        ("index_0", 0.0000, 1.5708),
        ("index_1", 0.0000, 1.7453),
    ],
}

# URDF `effort`, per motor. thumb_0 is the strong one; the rest are 1.4.
EFFORT = [2.45, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4]

# command_sender.py:117 `HandCommandSender`. Position gains, used when
# --q commands a pose. Motor 0 carries more load, hence the higher pair.
DEFAULT_KP = [2.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
DEFAULT_KD = [0.5, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]

# A hard ceiling on torque, well under the URDF efforts above: this script is
# for waking a finger, not for gripping.
TAU_CEILING = 0.5


def make_hand_mode(motor_index: int) -> int:
    """The Dex3 mode bitfield: id in [0..3], status in [4..6], timeout in 7.

    Transcribed from `decoupled_wbc/control/envs/g1/utils/command_sender.py`.
    A motor whose mode is left at 0 ignores the command.
    """
    return (motor_index & 0x0F) | (0x01 << 4) | (0x01 << 7)


# --- read ------------------------------------------------------------------


def format_state(msg, side="left") -> str:
    """One line per motor, plus the hand's own rails and error words."""
    side_names = [n for n, _, _ in LIMITS[side]]
    lines = []
    for i, m in enumerate(msg.motor_state[:NUM_MOTORS]):
        lines.append(
            f"  {i:d} {side_names[i]:9s} q {m.q:+8.4f}  dq {m.dq:+8.4f}  tau {m.tau_est:+7.3f}  "
            f"temp {m.temperature[0]:3d}/{m.temperature[1]:3d}  vol {m.vol:5.2f}  mode {m.mode:3d}"
        )
    lines.append(f"  power {msg.power_v:.2f} V {msg.power_a:.2f} A   error {list(msg.error)}")
    return "\n".join(lines)


def summarise(msg, baseline) -> str:
    """A one-line q vector plus how far each motor has moved from baseline."""
    q = [m.q for m in msg.motor_state[:NUM_MOTORS]]
    moved = max(abs(a - b) for a, b in zip(q, baseline)) if baseline else 0.0
    return "q [{}]  moved {:+.4f}".format(" ".join(f"{v:+7.4f}" for v in q), moved)


def read(args, out=print, clock=time.monotonic, sleep=time.sleep) -> int:
    latest = {}
    for side in args.sides:
        latest[side] = joint_sources.subscribe_dex3(side)
        out(f"subscribed  {joint_sources.DEX3_STATE_TOPICS[side]}")

    live = []
    for side in args.sides:
        if joint_sources.wait_for_sample(latest[side], args.wait):
            live.append(side)
            out(f"{side:5s}  publishing")
        else:
            out(f"{side:5s}  SILENT after {args.wait:.0f}s")
    if not live:
        out(
            "no dex3 state on any subscribed topic. Wrong --iface, hand "
            "unpowered, or the hand is not on this DDS domain."
        )
        return 1

    baseline = {}
    for side in live:
        msg = latest[side]()
        baseline[side] = [m.q for m in msg.motor_state[:NUM_MOTORS]]
        out(f"\n{side} first sample:\n{format_state(msg, side)}")

    out(f"\nwatching at {args.hz:.0f} Hz for {args.duration:.0f}s; move a finger by hand")
    period = 1.0 / args.hz
    end = clock() + args.duration
    while clock() < end:
        for side in live:
            out(f"{side:5s}  {summarise(latest[side](), baseline[side])}")
        sleep(period)

    for side in args.sides:
        out(f"{side:5s}  {latest[side].samples} samples")
        latest[side].close()
    return 0


# --- command ---------------------------------------------------------------


def publisher_for(side):
    from unitree_sdk2py.core.channel import ChannelPublisher
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_

    pub = ChannelPublisher(f"rt/dex3/{side}/cmd", HandCmd_)
    pub.Init()
    return pub


def blank_cmd():
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_

    return unitree_hg_msg_dds__HandCmd_()


def fill(cmd, motors, q, tau, kp, kd):
    """Write one command frame. Motors not in `motors` are left mode 0, off."""
    for i in range(NUM_MOTORS):
        m = cmd.motor_cmd[i]
        if i in motors:
            m.mode = make_hand_mode(i)
            m.q, m.dq, m.tau = q, 0.0, tau
            m.kp, m.kd = kp[i], kd[i]
        else:
            m.mode = 0
            m.q = m.dq = m.tau = m.kp = m.kd = 0.0
    return cmd


def command(args, out=print, clock=time.monotonic, sleep=time.sleep) -> int:
    side = args.sides[0]
    motors = set(args.motors)
    pure_torque = args.q is None
    kp = [0.0] * NUM_MOTORS if pure_torque else DEFAULT_KP
    kd = [0.0] * NUM_MOTORS if pure_torque else DEFAULT_KD

    latest = joint_sources.subscribe_dex3(side)
    if not joint_sources.wait_for_sample(latest, args.wait):
        out(
            f"refusing to command {side}: its state topic is silent, so nothing "
            "would show whether the fingers answered"
        )
        return 1
    out(f"{side} before:\n{format_state(latest(), side)}")

    out(
        "\nCOMMANDING rt/dex3/{}/cmd  motors {}  {}  for {:.1f}s".format(
            side,
            sorted(motors),
            f"tau {args.tau:+.3f} Nm, kp=kd=0"
            if pure_torque
            else f"q {args.q:+.3f} rad, kp/kd from command_sender",
            args.duration,
        )
    )
    if not args.yes:
        out("add --yes to actually publish. Nothing sent.")
        return 0

    pub = publisher_for(side)
    cmd = blank_cmd()
    period = 1.0 / args.hz
    target_q = 0.0 if pure_torque else args.q
    end = clock() + args.duration
    try:
        while clock() < end:
            # Ramp so a wrong sign shows before it is at full value.
            frac = min(1.0, (args.duration - (end - clock())) / args.ramp)
            fill(cmd, motors, target_q * frac, args.tau * frac, kp, kd)
            pub.Write(cmd)
            out(f"  t {frac:+5.2f}  {summarise(latest(), None)}")
            sleep(period)
    except KeyboardInterrupt:
        out("\ninterrupted")
    finally:
        # Release: mode 0 on every motor, published a few times in case one
        # frame is lost. A hand left holding torque cooks.
        for _ in range(10):
            pub.Write(fill(cmd, set(), 0.0, 0.0, kp, kd))
            sleep(0.01)
        out("released, all motors mode 0")

    out(f"\n{side} after:\n{format_state(latest(), side)}")
    latest.close()
    return 0


# --- cli -------------------------------------------------------------------


def finite_float(text: str) -> float:
    """argparse type: a float that is not NaN or inf. NaN passes every `abs(x) > limit` check."""
    value = float(text)
    if not math.isfinite(value):
        raise argparse.ArgumentTypeError(f"{text!r} is not a finite number")
    return value


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("mode", nargs="?", default="read", choices=("read", "command"))
    parser.add_argument(
        "--side",
        choices=SIDES + ("both",),
        default="both",
        help="read defaults to both; command needs one (default: %(default)s)",
    )
    parser.add_argument(
        "--iface",
        default=None,
        help="DDS network interface. enp4s0 on the lab machine; omit on the Jetson",
    )
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--hz", type=finite_float, default=5.0)
    parser.add_argument("--duration", type=finite_float, default=20.0)
    parser.add_argument(
        "--wait", type=finite_float, default=5.0, help="seconds to wait for a first sample"
    )
    parser.add_argument(
        "--motors",
        type=int,
        nargs="+",
        default=list(range(NUM_MOTORS)),
        help="command: motor indices to drive (default: all 7). See --limits",
    )
    parser.add_argument(
        "--limits", action="store_true", help="print the motor table for --side and exit"
    )
    parser.add_argument(
        "--tau", type=finite_float, default=0.0, help="command: feed-forward torque, Nm"
    )
    parser.add_argument(
        "--q",
        type=finite_float,
        default=None,
        help="command: position target, rad. Without it the command is pure torque, kp=kd=0",
    )
    parser.add_argument(
        "--ramp", type=finite_float, default=2.0, help="command: seconds to reach the full value"
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="command: actually publish. Without it the command mode prints what it would send",
    )
    args = parser.parse_args(argv)

    args.sides = list(SIDES) if args.side == "both" else [args.side]
    if args.mode == "command":
        if args.side == "both":
            parser.error("command takes one --side, not both")
        if abs(args.tau) > TAU_CEILING:
            parser.error(f"--tau above the {TAU_CEILING} Nm ceiling in this script")
        if args.q is not None:
            for m in args.motors:
                if not 0 <= m < NUM_MOTORS:
                    continue
                name, lo, hi = LIMITS[args.side][m]
                if not lo <= args.q <= hi:
                    parser.error(
                        f"--q {args.q:+.4f} is outside motor {m} ({args.side} {name}) range "
                        f"[{lo:+.4f}, {hi:+.4f}]"
                    )
        if args.q is None and args.tau == 0.0:
            parser.error("command with neither --q nor --tau does nothing")
        bad = [m for m in args.motors if not 0 <= m < NUM_MOTORS]
        if bad:
            parser.error(f"motor index out of 0..{NUM_MOTORS - 1}: {bad}")
    return args


def main(argv=None):
    args = parse_args(argv)

    if args.limits:
        for side in args.sides:
            print(f"{side} hand")
            for i, (name, lo, hi) in enumerate(LIMITS[side]):
                print(f"  {i:d}  {name:9s} [{lo:+.4f}, {hi:+.4f}] rad   {EFFORT[i]:.2f} Nm")
        return 0

    joint_sources.initialize_dds(args.iface, args.domain)
    print("dds      iface {}  domain {}".format(args.iface or "auto", args.domain))
    if args.mode == "read":
        return read(args)
    return command(args)


if __name__ == "__main__":
    sys.exit(main())
