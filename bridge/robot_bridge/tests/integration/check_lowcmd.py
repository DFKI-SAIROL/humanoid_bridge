"""Asserts the recorded /lowcmd matches what the driver asked for. Exit 1 on failure."""

import argparse
import json
import sys

from drive_gear_wbc import RAMP_DEFAULT, fingerprint, load_config
from fake_g1 import MODE_MACHINE

BRIDGE_HZ = 1000.0  # control_dt: 0.001 in G1_config.yaml
RAMP_SECONDS = 2.0  # `duration` in G1_config.yaml


def close(a, b, tol=1e-3):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def bridge_kp(path):
    import yaml

    with open(path) as handle:
        params = yaml.safe_load(handle)["robot_bridge"]["ros__parameters"]
    return [float(params[name]["kp"]) for name in params["joint_names"]]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("samples")
    parser.add_argument("--config", default=None)
    parser.add_argument("--params", default=None, help="G1_config.yaml; checks the timeout hold")
    args = parser.parse_args()

    with open(args.samples) as handle:
        samples = json.load(handle)
    config = load_config(args.config)
    n = config["NUM_MOTORS"]

    failures = []

    def check(name, ok, detail=""):
        print("{:<28} {}{}".format(name, "PASS" if ok else "FAIL", "  " + detail if detail else ""))
        if not ok:
            failures.append(name)

    check("received /lowcmd", len(samples) > 1000, f"{len(samples)} samples")
    if not samples:
        sys.exit(1)

    span = samples[-1]["t"] - samples[0]["t"]
    rate = len(samples) / span if span else 0.0
    check("publishes at ~1 kHz", 0.8 * BRIDGE_HZ < rate < 1.2 * BRIDGE_HZ, f"{rate:.0f} Hz")

    t0 = samples[0]["t"]
    ramped = min(samples, key=lambda s: abs((s["t"] - t0) - (RAMP_SECONDS + 0.3)))
    check(
        "start_control honoured",
        all(abs(v - RAMP_DEFAULT) < 1e-2 for v in ramped["q"][:n]),
        "q[0]={:.4f} wanted {}".format(ramped["q"][0], RAMP_DEFAULT),
    )

    # GEAR's stream ends in a command timeout, so the recording ends in hold
    # frames at the bridge's own gains. The last tracking frame carries GEAR's.
    tracking = [s for s in samples if close(s["kp"], config["MOTOR_KP"])]
    check("tracked GEAR's gains", bool(tracking), f"{len(tracking)} samples")
    last = tracking[-1] if tracking else samples[-1]
    want = fingerprint(n)
    check(
        "joint j lands on motor j",
        all(abs(a - b) < 1e-3 for a, b in zip(last["q"], want)),
        "q[:3]={}".format([round(v, 4) for v in last["q"][:3]]),
    )
    check("kp from config", all(abs(a - b) < 1e-3 for a, b in zip(last["kp"], config["MOTOR_KP"])))
    check("kd from config", all(abs(a - b) < 1e-3 for a, b in zip(last["kd"], config["MOTOR_KD"])))
    check("bridge forced mode 1", set(last["mode"]) == {1})
    # Not mode_pr: PR is 0, the field's default, so that check passed with or without the bridge.
    check(
        "bridge echoed mode_machine",
        last["mode_machine"] == MODE_MACHINE,
        "{} wanted {}".format(last["mode_machine"], MODE_MACHINE),
    )

    if args.params:
        tail = samples[-1]
        check(
            "timeout latches hold",
            close(tail["kp"], bridge_kp(args.params)) and close(tail["q"], want),
            "kp[0]={} q[0]={:.4f}".format(tail["kp"][0], tail["q"][0]),
        )

    if failures:
        print("\n{} check(s) failed: {}".format(len(failures), ", ".join(failures)))
        sys.exit(1)
    print("\nall checks passed")


if __name__ == "__main__":
    main()
