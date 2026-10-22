"""Drives a running G1_bridge against g1_sim and checks the simulated joints.

Exit 1 on failure. run_sim_bridge_test.sh starts the sim and the bridge first.
"""

import argparse
import json
import sys
import time
import urllib.request

import rclpy
import yaml
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, UInt16
from unitree_hg.msg import LowState

from g1_camera import wire
from g1_camera.client import CameraClient
from robot_bridge.cmd_client import RobotCmdClient

NUM_MOTORS = 29
KEY_R1, KEY_L1 = 1 << 0, 1 << 1
ELBOWS = (18, 25)
TOL = 0.05  # rad
RAMP_SECONDS = 3.0  # `duration` 2.0 plus margin


def load_params(path):
    with open(path) as handle:
        params = yaml.safe_load(handle)["robot_bridge"]["ros__parameters"]
    names = params["joint_names"]
    return {
        "kp": [float(params[n]["kp"]) for n in names],
        "kd": [float(params[n]["kd"]) for n in names],
        "ready_q": [float(v) for v in params["ready_q"]],
    }


class Driver:
    def __init__(self, node, params):
        self.node = node
        self.params = params
        self.client = RobotCmdClient(node, num_dof=NUM_MOTORS, control_frequency=50.0)
        self.client.set_default_cmd(params["ready_q"], params["kp"], params["kd"])
        self.keys = node.create_publisher(UInt16, "/g1_sim/keys", 10)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        node.create_subscription(String, "/bridge_state", self._on_state, latched)
        node.create_subscription(LowState, "/lowstate", self._on_lowstate, 10)
        self.state = None
        self.low = None
        self.failures = []

    def _on_state(self, msg):
        self.state = msg.data

    def _on_lowstate(self, msg):
        self.low = msg

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.005)

    def check(self, name, ok, detail=""):
        suffix = f"  ({detail})" if detail else ""
        print(f"{'PASS' if ok else 'FAIL'}  {name}{suffix}", flush=True)
        if not ok:
            self.failures.append(name)

    def error(self, target):
        q = [m.q for m in self.low.motor_state[:NUM_MOTORS]]
        return max(abs(a - b) for a, b in zip(q, target, strict=True))

    def check_camera(self):
        rgb = CameraClient(host="127.0.0.1").read_rgb()
        width, height = wire.DEFAULT_SIZE
        self.check("camera frame on ZMQ", rgb.shape == (height, width, 3), str(rgb.shape))
        url = f"http://127.0.0.1:{wire.DEFAULT_PREVIEW_PORT}"
        health = json.load(urllib.request.urlopen(f"{url}/health", timeout=2))
        self.check("camera preview /health", health["source"] == "g1_sim d435", str(health))
        jpeg = urllib.request.urlopen(f"{url}/frame.jpg", timeout=2).read()
        self.check("camera preview /frame.jpg", jpeg[:2] == b"\xff\xd8", f"{len(jpeg)} bytes")

    def stream(self, target, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.client.send_cmd(q_target_pos=target)
            self.spin(0.02)

    def run(self):
        self.spin(2.0)
        self.check("/lowstate from the sim", self.low is not None)
        self.check("bridge DISARMED", self.state == "DISARMED", str(self.state))
        if self.low is None:
            return
        self.check("remote header", bytes(self.low.wireless_remote[:2]) == b"\x55\x51")

        ready = self.params["ready_q"]
        reply = self.client.start_control(default_position=ready, timeout_sec=30.0)
        self.check("start_control", reply.success, reply.message)
        self.spin(RAMP_SECONDS)
        self.check("bridge ARMED", self.state == "ARMED", str(self.state))
        q = [m.q for m in self.low.motor_state[:NUM_MOTORS]]
        worst = max(range(NUM_MOTORS), key=lambda i: abs(q[i] - ready[i]))
        err = abs(q[worst] - ready[worst])
        self.check("holds ready_q", err < TOL, f"max error {err:.3f} rad at motor {worst}")

        target = list(ready)
        for i in ELBOWS:
            target[i] += 0.4
        self.stream(target, 3.0)
        err = self.error(target)
        self.check("/robot_cmd moves the elbows", err < TOL, f"max error {err:.3f} rad")

        self.check_camera()

        self.keys.publish(UInt16(data=KEY_L1 | KEY_R1))  # remote_combo_damp
        self.spin(0.5)
        self.keys.publish(UInt16(data=0))
        state = str(self.state)
        self.check("remote L1+R1 latches damp", state.startswith("LATCHED damp"), state)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("params", help="G1_config.yaml the bridge runs with")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("drive_sim")
    driver = Driver(node, load_params(args.params))
    try:
        driver.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if driver.failures:
        print(f"{len(driver.failures)} failed: {', '.join(driver.failures)}")
        sys.exit(1)
    print("all passed")


if __name__ == "__main__":
    main()
