"""Drives a T1_bridge against fake_t1.py: /robot_state, custom-mode arming, the gamepad e-stop.

Exit 1 on failure. run_fake_t1_test.sh starts the fake and the bridge first.
"""

import math
import sys
import time

import rclpy
import yaml
from booster_interface.msg import LowCmd
from bridge_interface.msg import RobotState
from fake_t1 import NUM_JOINT, RPY
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from robot_bridge.cmd_client import RobotCmdClient

RAMP_SECONDS = 2.5  # `duration` 2.0 plus margin
CUSTOM, DAMPING = '{"mode":3}', '{"mode":0}'


def quat(rpy):
    """w, x, y, z from ZYX roll, pitch, yaw."""
    cr, sr = math.cos(rpy[0] / 2), math.sin(rpy[0] / 2)
    cp, sp = math.cos(rpy[1] / 2), math.sin(rpy[1] / 2)
    cy, sy = math.cos(rpy[2] / 2), math.sin(rpy[2] / 2)
    return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]  # fmt: skip


class Driver:
    def __init__(self, node, params):
        self.node = node
        self.names = yaml.safe_load(open(params))["robot_bridge"]["ros__parameters"]["joint_names"]
        self.client = RobotCmdClient(node, num_dof=NUM_JOINT, control_frequency=50.0)
        self.buttons = node.create_publisher(String, "/fake_t1/buttons", 10)
        self.remote = node.create_publisher(Bool, "/fake_t1/remote", 10)
        latched = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)  # fmt: skip
        node.create_subscription(String, "/bridge_state", self._on_bridge, latched)
        node.create_subscription(String, "/fake_t1/mode", self._on_mode, latched)
        node.create_subscription(RobotState, "/robot_state", self._on_state, 10)
        node.create_subscription(LowCmd, "/joint_ctrl", self._on_cmd, 10)
        self.bridge = None
        self.modes = []
        self.state = None
        self.cmds = 0
        self.cmd_motors = None
        self.failures = []

    def _on_bridge(self, msg):
        self.bridge = msg.data

    def _on_mode(self, msg):
        self.modes.append(msg.data)

    def _on_state(self, msg):
        self.state = msg

    def _on_cmd(self, msg):
        self.cmds += 1
        self.cmd_motors = len(msg.motor_cmd)

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
        s = self.state
        names_ok = list(s.joint_names) == self.names
        self.check("joint_names = T1_config.yaml", names_ok, f"{len(s.q)} joints")
        self.check("the first 23 of 25 motors", len(s.q) == NUM_JOINT)
        q_ok = all(abs(a - b) < 1e-5 for a, b in zip(s.imu.quaternion, quat(RPY)))
        self.check("quaternion from rpy", q_ok, str([round(v, 4) for v in s.imu.quaternion]))
        self.check("gamepad connected", self.wait(lambda: self.state.remote.connected, 2.0))
        self.check("DISARMED", self.wait(lambda: self.bridge == "DISARMED", 3.0), str(self.bridge))

        self.remote.publish(Bool(data=False))
        self.wait(lambda: not self.state.remote.connected, 2.0)
        reply = self.client.start_control(default_position=list(self.state.q))
        self.check("no gamepad: arming refused", not reply.success, reply.message)
        self.remote.publish(Bool(data=True))
        self.wait(lambda: self.state.remote.connected, 2.0)

        reply = self.client.start_control(default_position=list(self.state.q))
        self.check("start_control at the measured pose", reply.success, reply.message)
        custom = self.wait(lambda: CUSTOM in self.modes, 2.0)
        self.check("custom mode requested", custom, str(self.modes))
        self.wait(lambda: False, RAMP_SECONDS)
        self.check("ARMED", self.bridge == "ARMED", str(self.bridge))
        self.check("/joint_ctrl flows, 23 motors", self.cmds > 100 and self.cmd_motors == NUM_JOINT,
                   f"{self.cmds} frames of {self.cmd_motors}")  # fmt: skip

        self.buttons.publish(String(data="RB+RT"))  # remote_combo_damp
        latched = self.wait(lambda: str(self.bridge).startswith("LATCHED damp"), 2.0)
        self.check("gamepad RB+RT latches damp", latched, str(self.bridge))
        self.buttons.publish(String(data=""))
        self.wait(lambda: False, 0.3)
        reply = self.client.estop_reset()
        self.check("estop_reset after the release", reply.success, reply.message)

        self.buttons.publish(String(data="LB+RB"))  # remote_combo_hold, from DISARMED: latches too
        held = self.wait(lambda: str(self.bridge).startswith("LATCHED hold"), 2.0)
        self.check("gamepad LB+RB latches hold", held, str(self.bridge))
        self.buttons.publish(String(data=""))
        rclpy.spin_once(self.node, timeout_sec=0.2)


def main():
    rclpy.init()
    node = rclpy.create_node("drive_t1")
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
