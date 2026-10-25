"""Drives a Go2_bridge against fake_go2.py: /robot_state, arming, the frame on the wire, e-stop.

Exit 1 on failure. run_fake_go2_test.sh starts the fake and the bridge first.
"""

import json
import sys
import time

import rclpy
import yaml
from bridge_interface.msg import RobotState
from fake_go2 import NUM_MOTORS
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, UInt16

from robot_bridge.cmd_client import RobotCmdClient

KEY_R1, KEY_R2 = 1 << 0, 1 << 4
RAMP_SECONDS = 3.5  # Go2_config.yaml `duration` 3.0 plus margin


class Driver:
    def __init__(self, node, params):
        self.node = node
        config = yaml.safe_load(open(params))["robot_bridge"]["ros__parameters"]
        self.names = config["joint_names"]
        self.ready_q = config["ready_q"]
        self.damp_kd = float(config["estop"]["damp_kd"])
        self.client = RobotCmdClient(node, num_dof=NUM_MOTORS, control_frequency=50.0)
        self.ready = node.create_client(__import__("std_srvs.srv", fromlist=["Trigger"]).Trigger,
                                        "/ready_position_control")  # fmt: skip
        self.keys = node.create_publisher(UInt16, "/fake_go2/keys", 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)  # fmt: skip
        node.create_subscription(String, "/bridge_state", self._on_bridge, latched)
        node.create_subscription(RobotState, "/robot_state", self._on_state, 10)
        node.create_subscription(String, "/fake_go2/frame", self._on_frame, 10)
        self.bridge = None
        self.state = None
        self.frame = None
        self.frames = 0
        self.bad_crc = 0
        self.failures = []

    def _on_bridge(self, msg):
        self.bridge = msg.data

    def _on_state(self, msg):
        self.state = msg

    def _on_frame(self, msg):
        self.frame = json.loads(msg.data)
        self.frames += 1
        self.bad_crc += 0 if self.frame["crc_ok"] else 1

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
        return predicate()

    def run(self):
        self.check("/robot_state arrives", self.wait(lambda: self.state is not None, 5.0))
        if self.state is None:
            return
        self.check("joint_names = Go2_config.yaml", list(self.state.joint_names) == self.names)
        self.check("the remote is connected", self.state.remote.connected)
        self.check("DISARMED", self.wait(lambda: self.bridge == "DISARMED", 3.0), str(self.bridge))

        reply = self.client.start_control(default_position=list(self.state.q))
        self.check("start_control at the lying pose", reply.success, reply.message)
        self.check("ARMED", self.wait(lambda: self.bridge == "ARMED", 3.0), str(self.bridge))
        flows = self.wait(lambda: self.frames > 20, 3.0)
        self.check("/lowcmd flows", flows, f"{self.frames} checked")
        f = self.frame or {}
        header = f.get("head") == [0xFE, 0xEF] and f.get("level_flag") == 0xFF
        self.check("head 0xFE 0xEF, level flag 0xFF", header)
        self.check("every slot in servo mode", f.get("modes") == [1] * 20)
        self.check("slots 12-19 stopped", all(q > 2e9 for q in f.get("spare_q", [])))
        bad = f"{self.bad_crc} bad of {self.frames}"
        self.check("every frame's CRC is right", self.bad_crc == 0, bad)

        future = self.ready.call_async(self.ready.srv_type.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.wait(lambda: False, RAMP_SECONDS)
        err = max(abs(a - b) for a, b in zip(self.state.q, self.ready_q, strict=True))
        self.check("Ready stands it up to ready_q", err < 0.05, f"max error {err:.3f} rad")

        self.keys.publish(UInt16(data=KEY_R1 | KEY_R2))  # remote_combo_damp
        latched = self.wait(lambda: str(self.bridge).startswith("LATCHED damp"), 2.0)
        self.check("remote R1+R2 latches damp", latched, str(self.bridge))
        self.keys.publish(UInt16(data=0))
        self.wait(lambda: False, 0.3)
        f = self.frame or {}
        damped = f.get("kp") == [0.0] * NUM_MOTORS and f.get("kd") == [self.damp_kd] * NUM_MOTORS
        self.check("damp frames: kp 0, kd damp_kd", damped, str(f.get("kd")))
        self.check("damp frames' CRC is right", self.bad_crc == 0, f"{self.bad_crc} bad")


def main():
    rclpy.init()
    node = rclpy.create_node("drive_go2")
    driver = Driver(node, sys.argv[1])
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
