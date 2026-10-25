"""Checks /robot_state and /hand_state of a running G1_bridge against fake_g1.py and fake_dex3.py.

Every field once: names and order from G1_config.yaml, values the fakes publish, the remote by name.
Exit 1 on failure.
"""

import math
import sys
import time

import rclpy
import yaml
from bridge_interface.msg import HandState, RobotState
from fake_g1 import ACC, GYRO, STICKS, tau_est
from std_msgs.msg import UInt16

KEY_R1, KEY_L2, KEY_A = 1 << 0, 1 << 5, 1 << 8


class Driver:
    def __init__(self, node, params, start_q):
        self.node = node
        self.config = yaml.safe_load(open(params))["robot_bridge"]["ros__parameters"]
        self.start_q = start_q
        self.keys = node.create_publisher(UInt16, "/fake_g1/keys", 10)
        node.create_subscription(RobotState, "/robot_state", self._on_state, 10)
        self.hands = {}
        for side in ("left", "right"):
            node.create_subscription(HandState, f"/hand_state/{side}", self._hand(side), 10)
        self.state = None
        self.count = 0
        self.failures = []

    def _on_state(self, msg):
        self.state = msg
        self.count += 1

    def _hand(self, side):
        def on(msg):
            self.hands[side] = msg

        return on

    def check(self, name, ok, detail=""):
        print(f"{name:<44} {'PASS' if ok else 'FAIL'}{'  ' + detail if detail else ''}", flush=True)
        if not ok:
            self.failures.append(name)

    def wait(self, predicate, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def run(self):
        self.check("/robot_state arrives", self.wait(lambda: self.state is not None, 5.0))
        if self.state is None:
            return
        start, n0 = time.monotonic(), self.count
        self.wait(lambda: False, 1.0)
        rate = (self.count - n0) / (time.monotonic() - start)
        self.check("at the robot's rate", rate > 200, f"{rate:.0f} Hz")

        s = self.state
        names = self.config["joint_names"]
        n = len(names)
        self.check("joint_names = G1_config.yaml", list(s.joint_names) == names, f"{n} joints")
        q_ok = all(abs(q - self.start_q) < 1e-6 for q in s.q)
        self.check("q from /lowstate", q_ok, f"q[0]={s.q[0]:.3f}")
        self.check("dq from /lowstate", len(s.dq) == len(names) and all(d == 0.0 for d in s.dq))
        self.check(
            "tau_est in joint order",
            len(s.tau_est) == n and all(abs(t - tau_est(i)) < 1e-6 for i, t in enumerate(s.tau_est)),
            f"tau_est[28]={s.tau_est[-1]:.3f}",
        )
        moved = self.wait(lambda: self.state.tick > s.tick, 1.0)
        self.check("tick moves", moved, f"tick {self.state.tick}")
        self.check("stamp set", s.stamp.sec > 0)
        close = lambda a, b: all(math.isclose(x, y, abs_tol=1e-6) for x, y in zip(a, b))  # noqa: E731
        self.check("imu quaternion", close(s.imu.quaternion, [1.0, 0.0, 0.0, 0.0]))
        self.check("imu rpy", close(s.imu.rpy, [0.0, 0.0, 0.0]))
        self.check("imu gyroscope", close(s.imu.gyroscope, GYRO))
        self.check("imu accelerometer", close(s.imu.accelerometer, ACC))
        r = s.remote
        self.check("remote connected", r.connected)
        self.check("no key pressed", list(r.pressed) == [])
        sticks = {"lx": r.lx, "ly": r.ly, "rx": r.rx, "ry": r.ry}
        self.check("sticks", sticks == STICKS, str(sticks))

        self.keys.publish(UInt16(data=KEY_R1 | KEY_L2 | KEY_A))  # no e-stop combo
        self.check(
            "pressed by name",
            self.wait(lambda: list(self.state.remote.pressed) == ["R1", "L2", "A"], 2.0),
            str(list(self.state.remote.pressed)),
        )
        self.keys.publish(UInt16(data=0))

        self.check("both /hand_state arrive", self.wait(lambda: len(self.hands) == 2, 5.0))
        for side in ("left", "right"):
            hand = self.hands.get(side)
            want = self.config[f"{side}_hand_joint_names"]
            self.check(
                f"/hand_state/{side} names",
                hand is not None and list(hand.joint_names) == want,
                f"{len(hand.joint_names) if hand else 0} joints",
            )
            self.check(
                f"/hand_state/{side} arrays",
                hand is not None and len(hand.q) == len(hand.dq) == len(hand.tau_est) == len(want),
            )


def main():
    params, start_q = sys.argv[1], float(sys.argv[2])
    rclpy.init()
    node = rclpy.create_node("drive_robot_state")
    driver = Driver(node, params, start_q)
    try:
        driver.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if driver.failures:
        print(f"FAILED: {', '.join(driver.failures)}")
        sys.exit(1)
    print("all checks passed")


if __name__ == "__main__":
    main()
