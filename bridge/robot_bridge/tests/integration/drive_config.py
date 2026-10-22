"""Starts G1_bridge on broken copies of a config: each must fail its parameter load.

Exit 1 on failure. run_config_test.sh pins DDS to loopback. No fake robot: a config
that loads waits for /lowstate, and is killed once its load is logged.
"""

import argparse
import copy
import os
import signal
import subprocess
import sys
import tempfile
import time

import yaml

LOADED = "E-stop: damp_kd"  # the last line of a load that passed
REFUSED = "Failed to load parameters."
TIMEOUT = 10.0  # s per start
R_ELBOW = 25
BRIDGE = ["ros2", "run", "robot_bridge", "G1_bridge", "--ros-args", "-r", "__node:=robot_bridge"]


def variants(params):
    """(name, text the refusal must log, change to the config) per broken copy."""
    names = params["joint_names"]

    def short_ready(p):
        p["ready_q"] = p["ready_q"][:-1]

    def ready_outside(p):
        p["ready_q"][R_ELBOW] = p[names[R_ELBOW]]["q_max"] + 0.5

    def idx_order(p):
        p["joint_names"] = [names[1], names[0]] + names[2:]

    def q_range(p):
        p[names[3]]["q_max"] = p[names[3]]["q_min"] - 0.1

    def tau_limit(p):
        p[names[3]]["tau_limit"] = -1.0

    def gain_range(p):
        p["upper_limbs_kp_max"] = p["upper_limbs_kp_min"] - 1.0

    def torque_control(p):
        p["torque_control"] = True

    def num_joint(p):
        p["num_joint"] = len(names) + 1

    return [
        ("ready_q one short", "ready_q has 28 values, expected 29", short_ready),
        ("ready_q outside its joint range", f"ready_q[{R_ELBOW}]", ready_outside),
        ("idx not its place in joint_names", "they must match", idx_order),
        ("q_max below q_min", "needs q_min <= q_max", q_range),
        ("negative tau_limit", "needs q_min <= q_max", tau_limit),
        ("inverted gain range", "gain range is inverted", gain_range),
        ("torque_control true", "Torque control is not supported", torque_control),
        ("num_joint not the joint count", "joint_names has 29 joints, num_joint is 30", num_joint),
    ]


def start(params_file, log_file, until=None):
    """Runs G1_bridge until it exits, logs `until`, or TIMEOUT. Returns (exit code or None, log)."""
    with open(log_file, "w") as log:
        proc = subprocess.Popen(
            BRIDGE + ["--params-file", params_file],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    end = time.monotonic() + TIMEOUT
    while time.monotonic() < end and proc.poll() is None:
        with open(log_file) as handle:
            if until and until in handle.read():
                break
        time.sleep(0.1)
    code = proc.poll()
    if code is None:
        # The group: `ros2 run` is a launcher, and killing it alone orphans G1_bridge.
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait()
    with open(log_file) as handle:
        return code, handle.read()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("params", help="a G1_config.yaml that loads")
    args = parser.parse_args()

    with open(args.params) as handle:
        config = yaml.safe_load(handle)
    out = tempfile.mkdtemp()
    failures = []

    def check(name, ok, detail=""):
        print(
            "{:<48} {}{}".format(name, "PASS" if ok else "FAIL", "  " + detail if detail else ""),
            flush=True,
        )
        if not ok:
            failures.append(name)

    # The copy, unchanged, must load: the round trip through yaml is not what refuses.
    path = os.path.join(out, "unchanged.yaml")
    with open(path, "w") as handle:
        yaml.safe_dump(config, handle)
    _, log = start(path, os.path.join(out, "unchanged.log"), until=LOADED)
    check("unchanged config loads", LOADED in log and REFUSED not in log)

    for i, (name, text, change) in enumerate(variants(config["robot_bridge"]["ros__parameters"])):
        broken = copy.deepcopy(config)
        change(broken["robot_bridge"]["ros__parameters"])
        path = os.path.join(out, f"broken{i}.yaml")
        with open(path, "w") as handle:
            yaml.safe_dump(broken, handle)
        code, log = start(path, os.path.join(out, f"broken{i}.log"))
        # Exit 1, not a signal: `ros2 run` reports an abort as 250.
        refused = code == 1 and REFUSED in log and text in log
        if code is None:
            why = f"still running after {TIMEOUT:.0f} s"
        else:
            why = f"exit {code}" + ("" if text in log else f", no '{text}'")
        check(f"{name}: refused", refused, "" if refused else f"{why}, log {out}/broken{i}.log")

    if failures:
        print("\n{} check(s) failed: {}".format(len(failures), ", ".join(failures)))
        sys.exit(1)
    print("\nall checks passed")


if __name__ == "__main__":
    main()
