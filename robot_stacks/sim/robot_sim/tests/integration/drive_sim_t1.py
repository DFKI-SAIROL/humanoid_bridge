"""Drives T1_bridge against robot_sim --robot t1: arming, hold, /robot_cmd, camera, e-stop.

Exit 1 on failure. run_sim_t1_test.sh starts the sim and the bridge (robot-bringup.sh) first.
"""

import json
import sys
import time
import urllib.request

import rclpy
import yaml
from bridge_interface.msg import RobotState
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from g1_camera import wire
from robot_bridge.cmd_client import RobotCmdClient

NUM_JOINT = 23
ELBOWS = (4, 8)  # left_elbow_pitch_joint, right_elbow_pitch_joint
TOL = 0.08  # rad; the arms' kp 10-50 (T1_config.yaml) sag under gravity
RAMP_SECONDS = 3.0  # `duration` 2.0 plus margin


class Driver:
    def __init__(self, node, params):
        self.node = node
        self.client = RobotCmdClient(node, num_dof=NUM_JOINT, control_frequency=50.0)
        config = yaml.safe_load(open(params))["robot_bridge"]["ros__parameters"]
        names = config["joint_names"]
        kp, kd = [float(config[n]["kp"]) for n in names], [float(config[n]["kd"]) for n in names]
        self.client.set_default_cmd(config["ready_q"], kp, kd)
        self.buttons = node.create_publisher(String, "/robot_sim/buttons", 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)  # fmt: skip
        node.create_subscription(String, "/bridge_state", self._on_bridge, latched)
        node.create_subscription(RobotState, "/robot_state", self._on_state, 10)
        self.bridge = None
        self.state = None
        self.failures = []

    def _on_bridge(self, msg):
        self.bridge = msg.data

    def _on_state(self, msg):
        self.state = msg

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
        return max(abs(a - b) for a, b in zip(self.state.q, target, strict=True))

    def stream(self, target, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.client.send_cmd(q_target_pos=target)
            self.spin(0.02)

    def run(self):
        self.spin(2.0)
        up = self.state is not None and len(self.state.q) == NUM_JOINT
        self.check("/robot_state from T1_bridge", up)
        if self.state is None:
            return
        self.check("the sim's gamepad is connected", self.state.remote.connected)
        self.check("DISARMED", self.bridge == "DISARMED", str(self.bridge))

        url = f"http://127.0.0.1:{wire.DEFAULT_PREVIEW_PORT}"
        health = json.load(urllib.request.urlopen(f"{url}/health", timeout=2))
        self.check("head camera /health", health["source"] == "robot_sim t1 head_rgb", str(health))
        jpeg = urllib.request.urlopen(f"{url}/frame.jpg", timeout=2).read()
        self.check("head camera /frame.jpg", jpeg[:2] == b"\xff\xd8", f"{len(jpeg)} bytes")

        measured = list(self.state.q)
        reply = self.client.start_control(default_position=measured)
        self.check("start_control at the measured pose", reply.success, reply.message)
        self.spin(RAMP_SECONDS)
        self.check("ARMED", self.bridge == "ARMED", str(self.bridge))
        err = self.error(measured)
        self.check("custom mode holds the measured pose", err < TOL, f"max error {err:.3f} rad")

        target = list(measured)
        for i in ELBOWS:
            target[i] -= 0.4
        self.stream(target, 3.0)
        err = self.error(target)
        self.check("/robot_cmd moves the elbows", err < TOL, f"max error {err:.3f} rad")

        self.buttons.publish(String(data="RB+RT"))  # remote_combo_damp
        self.spin(0.5)
        self.buttons.publish(String(data=""))
        latched = str(self.bridge).startswith("LATCHED damp")
        self.check("gamepad RB+RT latches damp", latched, str(self.bridge))


def main():
    rclpy.init()
    node = rclpy.create_node("drive_sim_t1")
    driver = Driver(node, sys.argv[1])
    try:
        driver.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if driver.failures:
        print(f"FAILED: {', '.join(driver.failures)}")
        sys.exit(1)
    print("all passed")


if __name__ == "__main__":
    main()
