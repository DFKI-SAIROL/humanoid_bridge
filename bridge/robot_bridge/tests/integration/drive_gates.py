"""Drives gates 1 and 2 of a fresh bridge against fake_g1.py --start-q and checks /lowcmd live.

Exit 1 on failure. run_fake_gates_test.sh starts the fake and the bridge first.
"""

import argparse
import sys
import time

import rclpy
import yaml
from unitree_hg.msg import LowCmd

from robot_bridge.cmd_client import RobotCmdClient

NUM_MOTORS = 29
L_HIP_PITCH, R_ELBOW = 0, 25
RAMP_SECONDS = 2.5  # `duration` 2.0 plus margin
WINDOW = 0.05  # s; speed over a window, so receive jitter and a dropped frame do not count


def load_params(path):
    with open(path) as handle:
        params = yaml.safe_load(handle)["robot_bridge"]["ros__parameters"]
    names = params["joint_names"]
    default = float(params.get("q_rate_limit_default", 0.0))
    return {
        "kp": [float(params[n]["kp"]) for n in names],
        "kd": [float(params[n]["kd"]) for n in names],
        "q_min": [float(params[n]["q_min"]) for n in names],
        "q_max": [float(params[n]["q_max"]) for n in names],
        "rate": [float(params[n].get("q_rate_limit", default)) for n in names],
    }


class Driver:
    def __init__(self, node, params):
        self.node = node
        self.params = params
        # Order 0: every command is a step, so only gate 2 can make it a ramp.
        self.client = RobotCmdClient(
            node, num_dof=NUM_MOTORS, control_frequency=50.0, interpolation_order=0.0
        )
        node.create_subscription(LowCmd, "/lowcmd", self._on_cmd, 100)
        self.frames = []  # (t, q)
        self.failures = []

    def _on_cmd(self, msg):
        self.frames.append((time.monotonic(), [m.q for m in list(msg.motor_cmd)[:NUM_MOTORS]]))

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.002)

    def check(self, name, ok, detail=""):
        print(
            "{:<48} {}{}".format(name, "PASS" if ok else "FAIL", "  " + detail if detail else ""),
            flush=True,
        )
        if not ok:
            self.failures.append(name)

    def send(self, q):
        self.client.send_cmd(
            q_target_pos=q,
            target_kp=self.params["kp"],
            target_kd=self.params["kd"],
            duration=0.02,
            hold_position=True,
        )

    def max_speed(self, frames, joint):
        """Fastest |dq/dt| over any WINDOW-long stretch of frames."""
        fastest, j = 0.0, 0
        for i, (t, q) in enumerate(frames):
            j = max(j, i)
            while j < len(frames) and frames[j][0] - t < WINDOW:
                j += 1
            if j == len(frames):
                break
            t2, q2 = frames[j]
            fastest = max(fastest, abs(q2[joint] - q[joint]) / (t2 - t))
        return fastest

    def run(self, start_q):
        p = self.params
        outside = [i for i in range(NUM_MOTORS) if not p["q_min"][i] <= start_q <= p["q_max"][i]]
        self.check("start pose inside every joint range", not outside, f"outside: {outside}")

        # The first arm after boot: lastCommand_ is all zeros until arming seeds it.
        first = len(self.frames)
        reply = self.client.start_control(default_position=[start_q] * NUM_MOTORS, timeout_sec=30.0)
        self.check("start_control", reply.success, reply.message)
        self.spin(RAMP_SECONDS)
        armed = self.frames[first:]
        worst = max((abs(v - start_q) for _, q in armed for v in q), default=None)
        self.check(
            "first arm after boot holds the measured pose",
            worst is not None and worst < 1e-3,
            f"{len(armed)} frames, worst |q - {start_q}| {worst}",
        )

        # Gate 2: a 1 rad step on an arm joint and a leg joint becomes a ramp.
        q = [start_q] * NUM_MOTORS
        q[R_ELBOW] = q[L_HIP_PITCH] = start_q - 1.0
        first = len(self.frames)
        self.send(q)
        self.spin(1.0)
        step = self.frames[first:]
        for name, joint in (("R_ELBOW", R_ELBOW), ("L_LEG_HIP_PITCH", L_HIP_PITCH)):
            rate = p["rate"][joint]
            speed = self.max_speed(step, joint)
            self.check(
                f"{name} step ramps at <= {rate} rad/s",
                rate > 0 and speed <= 1.25 * rate,
                f"max {speed:.2f} rad/s",
            )
            moving = [t for t, f in step if abs(f[joint] - start_q) > 1e-3]
            reached = [t for t, f in step if abs(f[joint] - q[joint]) < 1e-3]
            took = (reached[0] - moving[0]) if moving and reached else None
            self.check(
                f"{name} takes >= 1.0 / {rate} s",
                took is not None and took >= 0.85 / rate,
                f"took {took if took is None else round(took, 3)} s",
            )

        # Gate 1: a target past q_max stops at q_max.
        q[R_ELBOW] = 3.0
        first = len(self.frames)
        self.send(q)
        self.spin(1.5)
        clamp = [f[R_ELBOW] for _, f in self.frames[first:]]
        q_max = p["q_max"][R_ELBOW]
        highest = max(clamp) if clamp else float("nan")
        self.check(
            "R_ELBOW target 3.0 never passes q_max",
            bool(clamp) and highest <= q_max + 1e-4,
            f"max {highest:.4f}, q_max {q_max}",
        )
        self.check("R_ELBOW ends at q_max", bool(clamp) and abs(clamp[-1] - q_max) < 1e-3)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("params", help="the G1_config.yaml the bridge runs with")
    parser.add_argument("--start-q", type=float, default=0.2, help="the pose fake_g1 starts in")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("gates_driver")
    driver = Driver(node, load_params(args.params))
    try:
        driver.run(args.start_q)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if driver.failures:
        print("\n{} check(s) failed: {}".format(len(driver.failures), ", ".join(driver.failures)))
        sys.exit(1)
    print("\nall checks passed")


if __name__ == "__main__":
    main()
