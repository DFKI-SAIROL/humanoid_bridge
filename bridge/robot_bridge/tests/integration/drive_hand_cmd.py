"""Drives a running bridge's hand path through the GEAR hand adapter.

Streams a per-motor fingerprint to both hands, then optionally a command the
guards must reject.
"""

import argparse
import time

import numpy as np

# rclpy and the client are imported inside main(): check_hand_cmd imports the
# fingerprint from here to assert against it, and that is pure offline analysis
# which must not need a sourced ROS workspace.
NUM_HAND_JOINTS = 7
CONTROL_HZ = 100.0

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
        for _ in range(20):
            client.send_hand_cmd(
                "right",
                q=np.full(NUM_HAND_JOINTS, 9.0, dtype=np.float32),
                kp=np.ones(NUM_HAND_JOINTS, dtype=np.float32),
                kd=np.full(NUM_HAND_JOINTS, 0.2, dtype=np.float32),
                duration=1.0 / CONTROL_HZ,
            )
            time.sleep(1.0 / CONTROL_HZ)

    if args.send_nan:
        # Must be dropped whole: the last good command stays in force.
        print("drive: sending a NaN command", flush=True)
        bad = np.array(targets["right"], dtype=np.float32)
        bad[3] = np.nan
        for _ in range(20):
            client.send_hand_cmd(
                "right",
                q=bad,
                kp=np.ones(NUM_HAND_JOINTS, dtype=np.float32),
                kd=np.full(NUM_HAND_JOINTS, 0.2, dtype=np.float32),
                duration=1.0 / CONTROL_HZ,
            )
            time.sleep(1.0 / CONTROL_HZ)

    print("drive: done", flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
