"""Drives a running bridge through the GEAR adapter. Streams a per-joint fingerprint."""

import argparse
import time

import numpy as np
import rclpy

from robot_bridge.adapters.gear_wbc import make_factory
from robot_bridge.cmd_client import RobotCmdClient

NUM_MOTORS = 29
RAMP_DEFAULT = 0.123  # distinctive: ready_q_ starts [-0.1, 0.0, 0.0, 0.3]
CONTROL_HZ = 50.0


def fingerprint(n=NUM_MOTORS):
    """Joint j -> 0.01*(j+1). With an identity map, motor j must show the same."""
    return np.array([0.01 * (j + 1) for j in range(n)])


def synthetic_config(n=NUM_MOTORS):
    """Stand-in when GEAR is not installed. Identity maps, as g1_29dof has."""
    return {
        "NUM_MOTORS": n,
        "JOINT2MOTOR": list(range(n)),
        "MOTOR2JOINT": list(range(n)),
        "DEFAULT_MOTOR_ANGLES": [0.0] * n,
        "MOTOR_KP": [150.0] * 12 + [250.0] * 3 + [100.0] * 14,
        "MOTOR_KD": [2.0] * 12 + [5.0] * 3 + [2.0] * 14,
    }


def load_config(path):
    if not path:
        return synthetic_config()
    import yaml

    with open(path) as handle:
        return yaml.safe_load(handle)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", default=None, help="GEAR yaml; omit for a synthetic identity config"
    )
    parser.add_argument("--cycles", type=int, default=100)
    args = parser.parse_args()

    config = load_config(args.config)
    n = config["NUM_MOTORS"]

    rclpy.init()
    node = rclpy.create_node("gear_wbc_driver")
    client = RobotCmdClient(node, num_dof=n, control_frequency=CONTROL_HZ)
    adapter = make_factory(client)(config=config)  # exactly how g1_body builds it

    print("drive: calling start_control", flush=True)
    response = client.start_control(default_position=[RAMP_DEFAULT] * n, timeout_sec=30.0)
    # success is false only while an e-stop is latched; the ramp in /lowcmd is the real check.
    print("drive: start_control ->", response, flush=True)
    time.sleep(2.5)  # the bridge ramps over its `duration` parameter, 2.0 s

    target, zeros = fingerprint(n), np.zeros(n)
    print(f"drive: streaming {args.cycles} commands at {CONTROL_HZ:.0f} Hz", flush=True)
    for _ in range(args.cycles):
        adapter.send_command(target, zeros, zeros)
        time.sleep(1.0 / CONTROL_HZ)

    print("drive: done", flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
