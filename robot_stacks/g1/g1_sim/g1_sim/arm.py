"""Arm a G1_bridge whose robot is a sim: clear a stale latch, body at the measured pose, hands.

Loopback DDS only (guard.py). The lab's bridge path runs it in the G1 bench container:

    PYTHONPATH=$BRIDGE_CLONE/robot_stacks/g1/g1_sim:$BRIDGE_CLONE/bridge/robot_bridge \\
        python3 -m g1_sim.arm [--timeout 120] [--no-hands]

Exit 0 armed, 1 refused or not armed in time.
"""

import argparse
import os
import sys
import time

from g1_sim.guard import FIX, not_loopback

NUM_MOTORS = 29


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--timeout", type=float, default=120.0,
                        help="seconds to wait for /lowstate and for the bridge's services")
    parser.add_argument("--no-hands", action="store_true", help="body only")
    args = parser.parse_args(argv)
    refused = not_loopback(os.environ)
    if refused:
        sys.exit(f"g1_sim.arm refused: {refused}; {FIX}.")

    import rclpy
    from unitree_hg.msg import LowState

    from robot_bridge.cmd_client import RobotCmdClient

    rclpy.init()
    node = rclpy.create_node("g1_sim_arm")
    measured = {}

    def on_state(msg):
        measured["q"] = [float(msg.motor_state[i].q) for i in range(NUM_MOTORS)]

    node.create_subscription(LowState, "/lowstate", on_state, 10)
    end = time.monotonic() + args.timeout
    while "q" not in measured:
        if time.monotonic() > end:
            sys.exit(f"g1_sim.arm: no /lowstate within {args.timeout:.0f}s")
        rclpy.spin_once(node, timeout_sec=0.1)

    client = RobotCmdClient(node, num_dof=NUM_MOTORS, control_frequency=50.0)
    wait = max(1.0, end - time.monotonic())
    # A reused bridge latched state loss when the last run's sim stopped; success=false when nothing is latched.
    reply = client.estop_reset(timeout_sec=wait)
    print(f"estop_reset: {reply.success} {reply.message}", flush=True)
    reply = client.start_control(default_position=measured["q"], timeout_sec=wait)
    print(f"start_control at the measured pose: {reply.success} {reply.message}", flush=True)
    if not reply.success:
        return 1
    if not args.no_hands:
        reply = client.start_hand_control(timeout_sec=wait)
        print(f"start_hand_control: {reply.success} {reply.message}", flush=True)
        if not reply.success:
            return 1
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
