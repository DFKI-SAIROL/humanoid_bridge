"""Drives a running bridge's hand path through the GEAR hand adapter.

Streams a per-motor fingerprint to both hands, then optionally a command the
guards must reject.
"""

import argparse
import json
import time

import numpy as np

# rclpy and the client are imported inside main(): check_hand_cmd imports the
# fingerprint from here to assert against it, and that is pure offline analysis
# which must not need a sourced ROS workspace.
NUM_HAND_JOINTS = 7
CONTROL_HZ = 100.0
OVER_LIMIT = (9.0, 1.0, 0.2)  # q, kp, kd of the over-limit command
NAN_CMDS = 10  # 0.1 s: inside the last good command's watchdog (0.01 + 0.2 s)
PROBE_MODE_PR = 1  # --probe-lowcmd frames; the bridge always sends PR (0)

# The two hands are mirrored, so a legal angle on one is often illegal on the
# other. These are the signs each motor's range actually allows, in DDS order:
# thumb_0 is symmetric on both, and the rest flip.
SIGNS = {
    "left": (1, 1, 1, -1, -1, -1, -1),
    "right": (1, -1, -1, 1, 1, 1, 1),
}


def fingerprint(side, n=NUM_HAND_JOINTS):
    """Motor i -> 0.05*(i+1), signed so it stays inside that side's range.

    Distinct per motor, so a swapped index shows up. The largest magnitude is
    0.35, inside every range on both hands.
    """
    return np.array(
        [SIGNS[side][i] * 0.05 * (i + 1) for i in range(n)],
        dtype=np.float32,
    )


def check_estop(node, client, hands, targets):
    """Re-arm both hands, /estop_damp mid-stream: both get limp frames, re-arming is refused.

    Live on /dex3/*/cmd. Returns the names of the failed checks.
    """
    import rclpy
    from unitree_hg.msg import HandCmd

    frames = {side: [] for side in hands}  # (t, kp, kd)
    for side in hands:
        node.create_subscription(
            HandCmd,
            f"/dex3/{side}/cmd",
            lambda msg, side=side: frames[side].append(
                (time.monotonic(), [m.kp for m in msg.motor_cmd], [m.kd for m in msg.motor_cmd])
            ),
            100,
        )
    failures = []

    def check(name, ok, detail=""):
        print("{:<44} {}{}".format(name, "PASS" if ok else "FAIL", "  " + detail), flush=True)
        if not ok:
            failures.append(name)

    def stream(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            for side, adapter in hands.items():
                adapter.send_command(targets[side])
            tick = time.monotonic() + 1.0 / CONTROL_HZ
            while time.monotonic() < tick:
                rclpy.spin_once(node, timeout_sec=0.002)

    def since(t0, t1=float("inf")):
        return {side: [f for f in got if t0 <= f[0] < t1] for side, got in frames.items()}

    response = client.start_hand_control(timeout_sec=30.0)
    check("e-stop: start_hand_control", response.success, response.message)
    stream(0.5)
    live = since(time.monotonic() - 0.2)
    check(
        "e-stop: both hands live before it",
        all(got and all(any(k > 0.0 for k in f[1]) for f in got) for got in live.values()),
        ", ".join(f"{side} {len(got)} frames" for side, got in live.items()),
    )
    response = client.estop_damp()
    check("e-stop: /estop_damp", response.success, response.message)
    stop = time.monotonic()
    stream(0.5)
    # 50 ms: frames already in flight when the latch landed.
    after = since(stop + 0.05)
    check(
        "e-stop: both hands get limp frames (kp, kd 0)",
        all(
            len(got) >= 0.3 * CONTROL_HZ
            and all(not any(f[1]) and not any(f[2]) for f in got)
            for got in after.values()
        ),
        ", ".join(
            f"{side} {sum(not any(f[1]) and not any(f[2]) for f in got)} of {len(got)} limp"
            for side, got in after.items()
        ),
    )
    response = client.start_hand_control(timeout_sec=30.0)
    check(
        "e-stop: start_hand_control refused while latched",
        not response.success and "latched" in response.message,
        response.message,
    )
    response = client.estop_reset()
    check("e-stop: /estop_reset", response.success, response.message)
    return failures


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cycles", type=int, default=200)
    parser.add_argument(
        "--send-nan",
        action="store_true",
        help="finish with a NaN command, which the bridge must drop",
    )
    parser.add_argument(
        "--send-over-limit",
        action="store_true",
        help="finish with an out-of-range q, which the bridge must clamp",
    )
    parser.add_argument(
        "--check-lowcmd-guard",
        action="store_true",
        help="first check that start_hand_control is refused while another node publishes /lowcmd",
    )
    parser.add_argument(
        "--check-tilt-guard",
        action="store_true",
        help="first check that start_hand_control is refused while fake_g1 reports a tilt",
    )
    parser.add_argument(
        "--check-estop",
        action="store_true",
        help="then re-arm both hands and check that /estop_damp mid-stream releases both",
    )
    parser.add_argument(
        "--probe-lowcmd",
        action="store_true",
        help=f"last, publish /lowcmd frames with mode_pr {PROBE_MODE_PR}: fake_g1 must record them",
    )
    parser.add_argument(
        "--phases", help="write each phase's [start, end] in time.monotonic() to this JSON file"
    )
    args = parser.parse_args()

    import rclpy

    from robot_bridge.adapters.gear_hands import make_factory
    from robot_bridge.cmd_client import RobotCmdClient

    rclpy.init()
    node = rclpy.create_node("hand_cmd_driver")
    client = RobotCmdClient(node, num_dof=29, control_frequency=50.0)
    factory = make_factory(client)
    hands = {side: factory(is_left=(side == "left")) for side in ("left", "right")}

    if args.check_lowcmd_guard:
        from unitree_hg.msg import LowCmd

        # A restarted robot's motion service on /lowcmd blocks hand arming too.
        other = node.create_publisher(LowCmd, "/lowcmd", 10)
        time.sleep(1.0)  # until the bridge's graph sees it
        response = client.start_hand_control(timeout_sec=30.0)
        print("drive: start_hand_control, other /lowcmd publisher ->", response, flush=True)
        if response.success or "/lowcmd" not in response.message:
            raise SystemExit("start_hand_control not refused with another /lowcmd publisher")
        node.destroy_publisher(other)
        time.sleep(1.0)

    if args.check_tilt_guard:
        from std_msgs.msg import Float32

        # A body state fault blocks hand arming too: roll 1.5 rad, over imu_rpy_threshold (1.0).
        roll = node.create_publisher(Float32, "/fake_g1/roll", 10)
        time.sleep(1.0)  # until fake_g1 sees the publisher
        roll.publish(Float32(data=1.5))
        time.sleep(0.5)
        response = client.start_hand_control(timeout_sec=30.0)
        print("drive: start_hand_control, tilted ->", response, flush=True)
        if response.success or "imu" not in response.message:
            raise SystemExit("start_hand_control not refused on a tilt")
        roll.publish(Float32(data=0.0))
        time.sleep(0.5)
        node.destroy_publisher(roll)

    print("drive: calling start_hand_control", flush=True)
    response = client.start_hand_control(timeout_sec=30.0)
    print("drive: start_hand_control ->", response, flush=True)
    if not response.success:
        raise SystemExit(f"start_hand_control refused: {response.message}")
    time.sleep(2.5)  # the bridge ramps to hand_ready_q over `duration`, 2.0 s

    targets = {side: fingerprint(side) for side in hands}
    print(f"drive: streaming {args.cycles} commands at {CONTROL_HZ:.0f} Hz", flush=True)
    for _ in range(args.cycles):
        for side, adapter in hands.items():
            adapter.send_command(targets[side])
        time.sleep(1.0 / CONTROL_HZ)

    if args.send_over_limit:
        # Well past every q_max; the bridge must clamp rather than drop.
        print("drive: sending an out-of-range q", flush=True)
        q, kp, kd = OVER_LIMIT
        for _ in range(20):
            client.send_hand_cmd(
                "right",
                q=np.full(NUM_HAND_JOINTS, q, dtype=np.float32),
                kp=np.full(NUM_HAND_JOINTS, kp, dtype=np.float32),
                kd=np.full(NUM_HAND_JOINTS, kd, dtype=np.float32),
                duration=1.0 / CONTROL_HZ,
            )
            time.sleep(1.0 / CONTROL_HZ)

    phases = {}
    if args.send_nan:
        # Must be dropped whole: the last good command stays in force.
        print("drive: sending a NaN command", flush=True)
        bad = np.array(targets["right"], dtype=np.float32)
        bad[3] = np.nan
        phases["nan"] = [time.monotonic()]
        for _ in range(NAN_CMDS):
            client.send_hand_cmd(
                "right",
                q=bad,
                kp=np.ones(NUM_HAND_JOINTS, dtype=np.float32),
                kd=np.full(NUM_HAND_JOINTS, 0.2, dtype=np.float32),
                duration=1.0 / CONTROL_HZ,
            )
            time.sleep(1.0 / CONTROL_HZ)
        phases["nan"].append(time.monotonic())

    failures = []
    if args.check_estop:
        time.sleep(0.5)  # the right hand's watchdog first: the NaN window ends before the re-arm
        print("drive: e-stop with both hands live", flush=True)
        failures = check_estop(node, client, hands, targets)

    if args.probe_lowcmd:  # last: a second /lowcmd publisher refuses start_hand_control
        from unitree_hg.msg import LowCmd

        probe = node.create_publisher(LowCmd, "/lowcmd", 10)
        end = time.monotonic() + 5.0
        while probe.get_subscription_count() == 0 and time.monotonic() < end:
            time.sleep(0.05)
        time.sleep(0.5)  # the graph shows the reader before the DDS match completes
        print("drive: probing /lowcmd", flush=True)
        for _ in range(10):
            probe.publish(LowCmd(mode_pr=PROBE_MODE_PR))
            time.sleep(0.01)

    if args.phases:
        with open(args.phases, "w") as handle:
            json.dump(phases, handle)
    print("drive: done", flush=True)
    node.destroy_node()
    rclpy.shutdown()
    if failures:
        raise SystemExit("e-stop check(s) failed: " + ", ".join(failures))


if __name__ == "__main__":
    main()
