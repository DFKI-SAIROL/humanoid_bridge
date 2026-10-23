"""Drives G1_bridge + amo_controller against g1_sim: AMO must stand the G1 with the gantry off.

Exit 1 on failure. run_sim_amo_test.sh starts the sim, the bridge and the controller first.
"""

import sys
import time

import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String, UInt16
from unitree_hg.msg import LowState

from g1_stack.amo.goal import GoalPublisher, make_goal
from g1_stack.amo.policy import DEFAULT_DOF_POS, quat_to_euler
from robot_bridge.cmd_client import RobotCmdClient

NUM_MOTORS = 29
KEY_R1, KEY_L1, KEY_R2 = 1 << 0, 1 << 1, 1 << 4
HOLD_SECONDS = 3.0  # in the gantry, AMO driving, before the drop
STAND_SECONDS = 30.0
MAX_TILT = 0.35  # rad, roll or pitch; the bridge latches at 1.0
L_ELBOW = 18
ELBOW_STEP = 0.3  # rad on top of the default arm
ELBOW_TOL = 0.1


class Driver:
    def __init__(self, node):
        self.node = node
        self.client = RobotCmdClient(node, num_dof=NUM_MOTORS, control_frequency=50.0)
        self.gantry = node.create_publisher(Bool, "/g1_sim/gantry", 10)
        self.keys = node.create_publisher(UInt16, "/g1_sim/keys", 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String, "/bridge_state", self._on_state, latched)
        node.create_subscription(LowState, "/lowstate", self._on_lowstate, 10)
        self.state = None
        self.low = None
        self.states = set()
        self.failures = []

    def _on_state(self, msg):
        self.state = msg.data
        self.states.add(msg.data)

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

    def tilt(self):
        roll, pitch, _ = quat_to_euler(self.low.imu_state.quaternion)
        return max(abs(roll), abs(pitch))

    def watch_tilt(self, seconds, goals=None, goal=None):
        """The worst tilt over `seconds`; publishes `goal` at 30 Hz when given."""
        worst, end = 0.0, time.monotonic() + seconds
        while time.monotonic() < end:
            if goals is not None:
                goals.send(make_goal(**goal))
            self.spin(1 / 30)
            worst = max(worst, self.tilt())
        return worst

    def run(self):
        self.spin(2.0)
        self.check("/lowstate from the sim", self.low is not None)
        self.check("bridge DISARMED", self.state == "DISARMED", str(self.state))
        if self.low is None:
            return

        measured = [m.q for m in self.low.motor_state[:NUM_MOTORS]]
        reply = self.client.start_control(default_position=measured, timeout_sec=30.0)
        self.check("start_control from the measured pose", reply.success, reply.message)
        worst = self.watch_tilt(HOLD_SECONDS)
        self.check("bridge ARMED, AMO driving in the gantry", self.state == "ARMED", str(self.state))

        self.gantry.publish(Bool(data=False))
        worst = self.watch_tilt(STAND_SECONDS)
        self.check(f"AMO stands {STAND_SECONDS:.0f} s, gantry off", worst < MAX_TILT,
                   f"max tilt {worst:.3f} rad")
        self.check("bridge still ARMED", self.state == "ARMED", str(self.state))

        arm = DEFAULT_DOF_POS[15:].copy()
        arm[L_ELBOW - 15] += ELBOW_STEP
        goals = GoalPublisher()
        try:
            worst = self.watch_tilt(4.0, goals, {"arm_q": arm})
        finally:
            goals.close()
        err = abs(self.low.motor_state[L_ELBOW].q - arm[L_ELBOW - 15])
        self.check("the goal moves the left elbow", err < ELBOW_TOL, f"error {err:.3f} rad")
        self.check("still standing with the goal", worst < MAX_TILT, f"max tilt {worst:.3f} rad")

        self.gantry.publish(Bool(data=True))
        self.spin(1.0)
        self.keys.publish(UInt16(data=KEY_R1 | KEY_R2))  # remote_combo_damp
        self.spin(0.5)
        self.keys.publish(UInt16(data=0))
        state = str(self.state)
        self.check("remote R1+R2 latches damp", state.startswith("LATCHED damp"), state)
        print(f"note  bridge states seen: {sorted(self.states)}", flush=True)


def main():
    rclpy.init()
    node = rclpy.create_node("drive_amo_sim")
    driver = Driver(node)
    try:
        driver.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if driver.failures:
        print(f"FAILED: {', '.join(driver.failures)}")
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
