#!/usr/bin/env python3
"""Gantry check for gate 2: one target step on one joint, out and back; prints how fast the target moved.

MOVES THE ROBOT. Needs an ARMED bridge (tools/g1-bringup.sh --arm-from-measured) and the robot in the gantry.
Every other joint gets its measured pose; the step is a single /robot_cmd with interpolation_order 0,
so only the bridge's rate limit makes it a ramp.

    tools/g1-step-test.py --joint R_ELBOW --delta 0.5
"""

import argparse
import sys
import time
from pathlib import Path

import rclpy
import yaml
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from unitree_hg.msg import LowCmd, LowState

from robot_bridge.cmd_client import RobotCmdClient

NUM_MOTORS = 29
WINDOW = 0.05  # s; target speed over a window, so receive jitter does not count
DEFAULT_PARAMS = Path(__file__).resolve().parent.parent / "bridge/robot_bridge/params/G1_config.yaml"


def load(path):
    params = yaml.safe_load(open(path))["robot_bridge"]["ros__parameters"]
    names = params["joint_names"]
    default = float(params.get("q_rate_limit_default", 0.0))
    return names, params, default


def max_speed(samples):
    fastest, j = 0.0, 0
    for i, (t, q) in enumerate(samples):
        j = max(j, i)
        while j < len(samples) and samples[j][0] - t < WINDOW:
            j += 1
        if j == len(samples):
            break
        fastest = max(fastest, abs(samples[j][1] - q) / (samples[j][0] - t))
    return fastest


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--joint", default="R_ELBOW", help="config joint name (default R_ELBOW)")
    parser.add_argument("--delta", type=float, default=0.5, help="step size, rad (default 0.5)")
    parser.add_argument("--settle", type=float, default=1.5, help="seconds after each step (default 1.5)")
    parser.add_argument("--params", default=str(DEFAULT_PARAMS))
    args = parser.parse_args()

    names, params, default_rate = load(args.params)
    if args.joint not in names:
        print(f"unknown joint {args.joint}; one of: {' '.join(names)}", file=sys.stderr)
        return 2
    j = names.index(args.joint)
    joint = params[args.joint]
    rate = float(joint.get("q_rate_limit", default_rate))
    kp = [float(params[n]["kp"]) for n in names]
    kd = [float(params[n]["kd"]) for n in names]

    rclpy.init()
    node = rclpy.create_node("g1_step_test")
    state = {"bridge": None, "q": None}
    targets, measured = [], []

    def on_state(msg):
        state["q"] = [m.q for m in list(msg.motor_state)[:NUM_MOTORS]]
        measured.append((time.monotonic(), msg.motor_state[j].q))

    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, "/bridge_state", lambda m: state.update(bridge=m.data), latched)
    node.create_subscription(LowState, "/lowstate", on_state, qos_profile_sensor_data)
    node.create_subscription(LowCmd, "/lowcmd", lambda m: targets.append((time.monotonic(), m.motor_cmd[j].q)), 100)
    client = RobotCmdClient(node, num_dof=NUM_MOTORS, control_frequency=50.0, interpolation_order=0.0)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.002)

    spin(2.0)
    if state["bridge"] != "ARMED" or state["q"] is None:
        print(f"bridge state {state['bridge']!r}, /lowstate {'seen' if state['q'] else 'missing'}: "
              "arm first (tools/g1-bringup.sh --arm-from-measured --robot-is-clear)", file=sys.stderr)
        node.destroy_node(); rclpy.shutdown()
        return 2

    start = list(state["q"])
    goal = list(start)
    goal[j] = start[j] + args.delta
    if not float(joint["q_min"]) <= goal[j] <= float(joint["q_max"]):
        print(f"{args.joint}: {start[j]:.3f} + {args.delta} leaves [{joint['q_min']}, {joint['q_max']}]",
              file=sys.stderr)
        node.destroy_node(); rclpy.shutdown()
        return 2

    print(f"{args.joint}: {start[j]:.3f} -> {goal[j]:.3f} -> back; limit {rate} rad/s")
    results = []
    for label, q in (("out", goal), ("back", start)):
        first_t, first_m = len(targets), len(measured)
        client.send_cmd(q_target_pos=q, target_kp=kp, target_kd=kd, duration=0.02, hold_position=True)
        spin(args.settle)
        tgt, meas = targets[first_t:], measured[first_m:]
        results.append((label, max_speed(tgt), max_speed(meas), tgt[-1][1] if tgt else float("nan")))
    node.destroy_node()
    rclpy.shutdown()

    ok = True
    for label, target_speed, measured_speed, end in results:
        good = target_speed <= 1.25 * rate if rate > 0 else True
        ok &= good
        print(f"  {label:4s}  target max {target_speed:5.2f} rad/s  measured max {measured_speed:5.2f} rad/s  "
              f"target ends {end:.3f}  {'PASS' if good else 'FAIL'}")
    print(f"bridge state now: {state['bridge']}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
