"""Arms a running bridge, SIGINTs its process group, checks /lowcmd: damp for ~1 s, then silence.

Exit 1 on failure. run_fake_estop_test.sh runs it last: the bridge does not survive it.
"""

import argparse
import os
import signal
import sys
import time

import rclpy
from drive_estop import NUM_MOTORS, load_params
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from unitree_hg.msg import LowCmd

from robot_bridge.cmd_client import RobotCmdClient

DAMP_S = 1.0  # estop::SHUTDOWN_DAMP_S
GRACE = 0.05  # tracking frames already in flight when the signal lands


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("params", help="the G1_config.yaml the bridge runs with")
    parser.add_argument("pgid", type=int, help="the bridge's process group (setsid)")
    parser.add_argument("bridge_log", help="the bridge's log, read for its exit line")
    args = parser.parse_args()
    params = load_params(args.params)

    rclpy.init()
    node = rclpy.create_node("shutdown_driver")
    client = RobotCmdClient(node, num_dof=NUM_MOTORS, control_frequency=50.0)
    frames, states, failures = [], [], []  # frames: (t, kp, kd)

    def on_cmd(msg):
        motors = list(msg.motor_cmd)[:NUM_MOTORS]
        frames.append((time.monotonic(), [m.kp for m in motors], [m.kd for m in motors]))

    latched = QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
    )
    node.create_subscription(LowCmd, "/lowcmd", on_cmd, 10)
    node.create_subscription(String, "/bridge_state", lambda msg: states.append(msg.data), latched)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.005)

    def check(name, ok, detail=""):
        print(
            "{:<44} {}{}".format(name, "PASS" if ok else "FAIL", "  " + detail if detail else ""),
            flush=True,
        )
        if not ok:
            failures.append(name)

    reply = client.start_control(default_position=[0.123] * NUM_MOTORS, timeout_sec=30.0)
    check("start_control", reply.success, reply.message)
    spin(0.5)
    tracking = bool(frames) and all(abs(k - w) < 1e-3 for k, w in zip(frames[-1][1], params["kp"]))
    check("tracking frames before SIGINT", tracking)

    sent = time.monotonic()
    os.killpg(args.pgid, signal.SIGINT)  # what Ctrl-C in its terminal sends
    spin(3.0)

    after = [f for f in frames if f[0] > sent + GRACE]
    bad = sum(
        any(abs(k) > 1e-6 for k in kp) or any(abs(d - params["damp_kd"]) > 1e-3 for d in kd)
        for _, kp, kd in after
    )
    check(
        "every frame after SIGINT is damp",
        bool(after) and bad == 0,
        f"{bad} of {len(after)} not kp 0",
    )
    check("damp frames keep coming", len(after) > 0.5 * DAMP_S * 1000, f"{len(after)} frames")
    span = frames[-1][0] - sent if frames else 0.0
    check(
        "damp for about 1 s",
        0.8 * DAMP_S <= span <= 1.5 * DAMP_S,
        f"last frame {span:.2f} s after SIGINT",
    )
    check("then silent", bool(frames) and time.monotonic() - frames[-1][0] > 1.0)
    check(
        "/bridge_state LATCHED damp shutdown", "LATCHED damp shutdown" in states, f"{states[-2:]}"
    )

    end = time.monotonic() + 5.0
    while time.monotonic() < end and "G1 Bridge stopped." not in open(args.bridge_log).read():
        time.sleep(0.1)
    check("bridge exits", "G1 Bridge stopped." in open(args.bridge_log).read())

    node.destroy_node()
    rclpy.shutdown()
    if failures:
        print("\n{} check(s) failed: {}".format(len(failures), ", ".join(failures)))
        sys.exit(1)
    print("\nall checks passed")


if __name__ == "__main__":
    main()
