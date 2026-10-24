"""Ready opens the Dex3 hands: /ready_position_control ramps each hand with state to hand_ready_q and holds.

Live on /dex3/*/cmd. run_fake_hand_ready_test.sh starts the fakes and the bridge first. Exit 1 on failure.
"""

import argparse
import time

import numpy as np
import yaml

from drive_hand_cmd import fingerprint

SIDES = ("left", "right")
RAMP_SECONDS = 2.5  # `duration` 2.0 plus margin
HOLD_SECONDS = 1.0  # past the 0.2 s watchdog a non-holding command would get
TOL = 1e-3  # rad; the fake tracks perfectly
NO_JUMP = 0.05  # rad; the first frame after Ready, from the held pose


def load_hand_params(path):
    with open(path) as handle:
        params = yaml.safe_load(handle)["robot_bridge"]["ros__parameters"]
    return {
        "ready_q": np.array(params["hand_ready_q"], dtype=np.float32),
        "kp": {
            side: np.array([params[n]["kp"] for n in params[f"{side}_hand_joint_names"]], dtype=np.float32)
            for side in SIDES
        },
    }


class Driver:
    def __init__(self, node, client, params, silent):
        import rclpy
        from unitree_hg.msg import HandCmd

        self.rclpy = rclpy
        self.node = node
        self.client = client
        self.params = params
        self.silent = silent
        self.live = [side for side in SIDES if side != silent]
        self.frames = {side: [] for side in SIDES}  # (t, q, kp)
        for side in SIDES:
            node.create_subscription(
                HandCmd,
                f"/dex3/{side}/cmd",
                lambda msg, side=side: self.frames[side].append(
                    (
                        time.monotonic(),
                        np.array([m.q for m in msg.motor_cmd], dtype=np.float32),
                        np.array([m.kp for m in msg.motor_cmd], dtype=np.float32),
                    )
                ),
                100,
            )
        self.failures = []

    def check(self, name, ok, detail=""):
        print("{:<52} {}{}".format(name, "PASS" if ok else "FAIL", "  " + detail), flush=True)
        if not ok:
            self.failures.append(name)

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.002)

    def since(self, side, t0, t1=float("inf")):
        return [f for f in self.frames[side] if t0 <= f[0] < t1]

    def ready(self, label):
        """Call Ready; check the reply names the hands. Returns the call time."""
        t0 = time.monotonic()
        reply = self.client.ready_position_control(timeout_sec=30.0)
        want = "hands: " + ", ".join(self.live)
        if self.silent:
            want += "; silent, not started: " + self.silent
        self.check(f"{label}: ready_position_control", reply.success, reply.message)
        self.check(f"{label}: reply names the hands", reply.message.endswith(want), f"want '...{want}'")
        return t0

    def check_opened(self, label, t0):
        """Every live hand at hand_ready_q with config kp, through the hold; no frame for the silent one."""
        self.spin(RAMP_SECONDS + HOLD_SECONDS)
        held_from = t0 + RAMP_SECONDS
        for side in self.live:
            held = self.since(side, held_from)
            err = max((float(np.max(np.abs(f[1] - self.params["ready_q"]))) for f in held), default=np.inf)
            self.check(f"{label}: {side} holds hand_ready_q", err < TOL, f"{len(held)} frames, max error {err:.4f}")
            gains = all(np.allclose(f[2], self.params["kp"][side]) for f in held)
            self.check(f"{label}: {side} holds at config kp", bool(held) and gains)
        if self.silent:
            got = self.since(self.silent, t0)
            self.check(f"{label}: silent {self.silent} hand gets no frame", not got, f"{len(got)} frames")

    def run(self):
        self.spin(1.0)

        # 1. Hands not armed: Ready arms them, from the measured q at zero gain.
        t0 = self.ready("unarmed")
        self.check_opened("unarmed", t0)
        for side in self.live:
            first = self.since(side, t0)[:1]
            self.check(
                f"unarmed: {side} gains ramp up from 0",
                bool(first) and float(np.max(first[0][2])) < 0.5 * float(np.max(self.params["kp"][side])),
                f"first kp {first[0][2].tolist() if first else None}",
            )

        # 2. A client holds the hands closed: Ready overrides it, gains never drop.
        targets = {side: fingerprint(side) for side in self.live}
        for side in self.live:
            self.client.send_hand_cmd(
                side, q=targets[side], kp=self.params["kp"][side],
                kd=np.full(7, 0.2, dtype=np.float32), duration=0.5, hold_position=True,
            )
        self.spin(1.0)
        for side in self.live:
            last = self.frames[side][-1:]
            err = float(np.max(np.abs(last[0][1] - targets[side]))) if last else np.inf
            self.check(f"client: {side} held at the fingerprint", err < TOL, f"max error {err:.4f}")

        t0 = self.ready("client")
        self.check_opened("client", t0)
        for side in self.live:
            ramp = self.since(side, t0, t0 + RAMP_SECONDS)
            jump = float(np.max(np.abs(ramp[0][1] - targets[side]))) if ramp else np.inf
            self.check(f"client: {side} ramps from the held pose", jump < NO_JUMP, f"first step {jump:.4f}")
            slack = sum(not np.all(f[2] > 0.0) for f in ramp)
            self.check(f"client: {side} never slack in the ramp", bool(ramp) and not slack, f"{slack} frames")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("params", help="G1_config.yaml the bridge runs with")
    parser.add_argument("--silent", choices=SIDES, help="the hand fake_dex3 runs --silent")
    args = parser.parse_args()

    import rclpy

    from robot_bridge.cmd_client import RobotCmdClient

    rclpy.init()
    node = rclpy.create_node("hand_ready_driver")
    client = RobotCmdClient(node, num_dof=29, control_frequency=50.0)
    driver = Driver(node, client, load_hand_params(args.params), args.silent)
    try:
        driver.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if driver.failures:
        raise SystemExit(f"{len(driver.failures)} failed: " + ", ".join(driver.failures))
    print("all checks passed", flush=True)


if __name__ == "__main__":
    main()
