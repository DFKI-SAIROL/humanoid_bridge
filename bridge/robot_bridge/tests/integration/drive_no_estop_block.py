"""A G1 bridge whose config has no estop: block still refuses arming without the remote.

Exit 1 on failure. run_fake_gates_test.sh starts the bridge on G1_config.yaml minus that block.
"""

import argparse
import sys
import time

import rclpy
from std_msgs.msg import Bool

from robot_bridge.cmd_client import RobotCmdClient

NUM_MOTORS = 29


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bridge_log", help="the bridge's log, read for its default line")
    parser.add_argument("--start-q", type=float, default=0.2, help="the pose fake_g1 holds")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("no_estop_driver")
    client = RobotCmdClient(node, num_dof=NUM_MOTORS, control_frequency=50.0)
    remote = node.create_publisher(Bool, "/fake_g1/remote", 10)
    failures = []

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)

    def check(name, ok, detail=""):
        print(
            "{:<48} {}{}".format(name, "PASS" if ok else "FAIL", "  " + detail if detail else ""),
            flush=True,
        )
        if not ok:
            failures.append(name)

    up = client.start_control_client.wait_for_service(timeout_sec=30.0)
    check("bridge up on a config without estop:", up)
    end = time.monotonic() + 5.0
    while up and remote.get_subscription_count() == 0 and time.monotonic() < end:
        spin(0.05)  # a remote switch published before discovery matched is lost
    if up:
        with open(args.bridge_log) as handle:
            log = handle.read()
        check("bridge logs the G1 default", "No estop.require_remote in the config" in log)
        check(
            "e-stop line says arming needs the remote",
            "arming needs the remote" in log and "does not need the remote" not in log,
        )
        remote.publish(Bool(data=False))
        spin(1.0)
        reply = client.start_control(default_position=[args.start_q] * NUM_MOTORS, timeout_sec=30.0)
        check(
            "start_control refused without the remote",
            not reply.success and "remote" in reply.message,
            reply.message,
        )
        remote.publish(Bool(data=True))
        spin(0.5)
        reply = client.start_control(default_position=[args.start_q] * NUM_MOTORS, timeout_sec=30.0)
        check("start_control with the remote", reply.success, reply.message)

    node.destroy_node()
    rclpy.shutdown()
    if failures:
        print("\n{} check(s) failed: {}".format(len(failures), ", ".join(failures)))
        sys.exit(1)
    print("\nall checks passed")


if __name__ == "__main__":
    main()
