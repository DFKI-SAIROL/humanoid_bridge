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
from rcl_interfaces.srv import GetParameters
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, UInt16
from unitree_hg.msg import HandState, LowState

from g1_camera import wire
from g1_camera.client import CameraClient
from g1_sim.model import DEFAULT_MJCF
from robot_bridge.cmd_client import RobotCmdClient

NUM_MOTORS = 29
KEY_R1, KEY_L1, KEY_R2 = 1 << 0, 1 << 1, 1 << 4
ELBOWS = (18, 25)
TOL = 0.05  # rad
RAMP_SECONDS = 3.0  # `duration` 2.0 plus margin
HAND_SIDES = ("left", "right")
HAND_TOL = 0.1  # rad; finger PD at kp 1-2 against the MJCF


def load_params(path):
    with open(path) as handle:
        params = yaml.safe_load(handle)["robot_bridge"]["ros__parameters"]
    names = params["joint_names"]
    return {
        "kp": [float(params[n]["kp"]) for n in names],
        "kd": [float(params[n]["kd"]) for n in names],
        "q_min": [float(params[n]["q_min"]) for n in names],
        "q_max": [float(params[n]["q_max"]) for n in names],
        "ready_q": [float(v) for v in params["ready_q"]],
        "hand_ready_q": [float(v) for v in params["hand_ready_q"]],
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
        self.hands = {}  # side -> latest HandState; empty without a Dex3 model
        for side in HAND_SIDES:
            node.create_subscription(
                HandState, f"/dex3/{side}/state", lambda msg, s=side: self.hands.__setitem__(s, msg), 10
            )
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

    def check_hands_open(self, message):
        """Ready opened the Dex3 hands, or named none on a model without them."""
        if not self.hands:
            want = "hands: none (no state)"
            self.check("Ready without Dex3 names no hand", message.endswith(want), message)
            return
        self.check("Ready names both hands", message.endswith("hands: left, right"), message)
        want = self.params["hand_ready_q"]
        for side in HAND_SIDES:
            q = [m.q for m in self.hands[side].motor_state[: len(want)]]
            err = max(abs(a - b) for a, b in zip(q, want, strict=True))
            self.check(f"{side} hand open at hand_ready_q", err < HAND_TOL, f"max error {err:.3f} rad")

    def check_mjcf(self):
        client = self.node.create_client(GetParameters, "/g1_sim/get_parameters")
        client.wait_for_service(timeout_sec=5.0)
        future = client.call_async(GetParameters.Request(names=["mjcf"]))
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        got = future.result().values[0].string_value if future.result() else None
        want = str(DEFAULT_MJCF.resolve())
        self.check("parameter mjcf is the loaded file", got == want, f"{got}, want {want}")

    def run(self):
        self.spin(2.0)
        self.check_mjcf()
        self.check("/lowstate from the sim", self.low is not None)
        self.check("bridge DISARMED", self.state == "DISARMED", str(self.state))
        if self.low is None:
            return
        self.check("remote header", bytes(self.low.wireless_remote[:2]) == b"\x55\x51")

        # As the commander arms: to the measured pose, then Ready.
        measured = [m.q for m in self.low.motor_state[:NUM_MOTORS]]
        lo, hi = self.params["q_min"], self.params["q_max"]
        past = [i for i in range(NUM_MOTORS) if not lo[i] <= measured[i] <= hi[i]]
        print(f"note  measured joints past their range: {past}", flush=True)
        reply = self.client.start_control(default_position=measured, timeout_sec=30.0)
        self.check("start_control from the measured pose", reply.success, reply.message)
        self.spin(RAMP_SECONDS)
        self.check("bridge ARMED", self.state == "ARMED", str(self.state))

        ready = self.params["ready_q"]
        reply = self.client.ready_position_control()
        self.check("ready_position_control", reply.success, reply.message)
        self.spin(RAMP_SECONDS)
        q = [m.q for m in self.low.motor_state[:NUM_MOTORS]]
        worst = max(range(NUM_MOTORS), key=lambda i: abs(q[i] - ready[i]))
        err = abs(q[worst] - ready[worst])
        self.check("holds ready_q", err < TOL, f"max error {err:.3f} rad at motor {worst}")
        self.check_hands_open(reply.message)

        target = list(ready)
        for i in ELBOWS:
            target[i] += 0.4
        self.stream(target, 3.0)
        err = self.error(target)
        self.check("/robot_cmd moves the elbows", err < TOL, f"max error {err:.3f} rad")

        self.check_camera()

        self.keys.publish(UInt16(data=KEY_R1 | KEY_R2))  # remote_combo_damp
        self.spin(0.5)
        self.keys.publish(UInt16(data=0))
        state = str(self.state)
        self.check("remote R1+R2 latches damp", state.startswith("LATCHED damp"), state)


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
