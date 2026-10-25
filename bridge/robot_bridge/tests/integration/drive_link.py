"""Drives /robot_link of a running bridge against fake_g1.py: link drops and reboots, armed and not.

Exit 1 on failure. run_fake_link_test.sh runs `armed` on one bridge, then `disarmed` on a fresh one:
a restart is final for a bridge.
"""

import argparse
import sys
import time

import rclpy
from drive_estop import NUM_MOTORS
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger
from unitree_hg.msg import LowCmd

from robot_bridge.cmd_client import RobotCmdClient

RAMP_SECONDS = 2.5  # `duration` 2.0 plus margin
RESTARTED_WHY = "robot restarted: restart the bridge to release /lowcmd"


class Driver:
    def __init__(self, node):
        self.node = node
        self.client = RobotCmdClient(node, num_dof=NUM_MOTORS, control_frequency=50.0)
        self.ready = node.create_client(Trigger, "/ready_position_control")
        self.pause = node.create_publisher(Bool, "/fake_g1/pause", 10)
        self.reboot = node.create_publisher(Float32, "/fake_g1/reboot", 10)
        self.release = node.create_publisher(Bool, "/fake_g1/release", 10)
        latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        node.create_subscription(String, "/bridge_state", self._on_state, latched)
        node.create_subscription(String, "/robot_link", self._on_link, latched)
        node.create_subscription(LowCmd, "/lowcmd", self._on_cmd, 10)
        self.state = None
        self.link = None
        self.links = []  # every /robot_link, in order
        self.count = 0
        self.failures = []

    def _on_state(self, msg):
        self.state = msg.data

    def _on_link(self, msg):
        self.link = msg.data
        self.links.append(msg.data)

    def _on_cmd(self, _msg):
        self.count += 1

    # --- helpers ---------------------------------------------------------

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.005)

    def wait(self, predicate, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if predicate():
                return True
            rclpy.spin_once(self.node, timeout_sec=0.005)
        return predicate()

    def check(self, name, ok, detail=""):
        print(
            "{:<44} {}{}".format(name, "PASS" if ok else "FAIL", "  " + detail if detail else ""),
            flush=True,
        )
        if not ok:
            self.failures.append(name)

    def check_state(self, name, wanted, timeout=3.0):
        self.check(name, self.wait(lambda: self.state == wanted, timeout), f"state '{self.state}'")

    def check_link(self, name, wanted, timeout=3.0):
        self.check(name, self.wait(lambda: self.link == wanted, timeout), f"link '{self.link}'")

    def gap(self, prefix, low, high, timeout=5.0):
        """Waits for a /robot_link starting with prefix; True if its gap is in [low, high]."""
        self.wait(lambda: (self.link or "").startswith(prefix), timeout)
        parts = (self.link or "").split()
        ok = len(parts) >= 2 and parts[0] == prefix and low <= float(parts[1]) <= high
        return ok, f"link '{self.link}', wanted {prefix} {low}-{high} s"

    def refused(self, name, reply, why=RESTARTED_WHY):
        self.check(name, not reply.success and why in reply.message, reply.message)

    def start(self, q):
        return self.client.start_control(default_position=[q] * NUM_MOTORS, timeout_sec=30.0)

    def up(self):
        ok = self.client.start_control_client.wait_for_service(timeout_sec=30.0)
        self.check("bridge up: /start_control", ok)
        inputs = (self.pause, self.reboot, self.release)
        matched = self.wait(lambda: all(p.get_subscription_count() for p in inputs), 5.0)
        self.check("fake_g1 inputs matched", matched)
        self.check_link("a new bridge starts OK", "OK")
        self.check_state("starts DISARMED", "DISARMED")
        return ok and matched

    # --- the runs --------------------------------------------------------

    def armed(self):
        if not self.up():
            return

        # Link drop while disarmed: LOST, then BACK; nothing latches; arming clears it.
        self.pause.publish(Bool(data=True))
        self.check("disarmed drop: LOST", *self.gap("LOST", 0.2, 0.3))
        self.spin(0.3)
        self.pause.publish(Bool(data=False))
        self.check("disarmed drop: BACK, gap ~0.5 s", *self.gap("BACK", 0.4, 0.9))
        self.check("BACK names no restart", (self.link or "").endswith(" no_restart"), self.link)
        self.check_state("disarmed drop latches nothing", "DISARMED")
        self.check("start_control after BACK", self.start(0.123).success)
        self.check_link("arming clears BACK -> OK", "OK")
        self.spin(RAMP_SECONDS)
        self.check_state("ARMED", "ARMED")

        # Link drop while armed: state_loss latches and goes silent; BACK; a reset clears it.
        self.links.clear()
        self.pause.publish(Bool(data=True))
        self.check_state("armed drop latches damp state_loss", "LATCHED damp state_loss")
        self.check_state("then silent", "LATCHED damp state_loss silent")
        self.spin(0.5)
        lost = [link for link in self.links if link.startswith("LOST")]
        self.check("LOST again once a second", len(lost) >= 2 and lost[1].startswith("LOST 1."), f"{lost}")
        self.pause.publish(Bool(data=False))
        self.check("armed drop: BACK, gap ~1.7 s", *self.gap("BACK", 1.5, 2.3))
        self.check("BACK names no restart", (self.link or "").endswith(" no_restart"), self.link)
        self.check_state("BACK keeps the latch", "LATCHED damp state_loss silent")
        reply = self.client.estop_reset()
        self.check("estop_reset after BACK", reply.success, reply.message)
        self.check_link("reset clears BACK -> OK", "OK")
        self.check_state("reset from silent -> DISARMED", "DISARMED")

        # Reboot while armed: robot_restart, final for this bridge.
        self.check("start_control", self.start(0.123).success)
        self.spin(RAMP_SECONDS)
        self.check_state("ARMED", "ARMED")
        self.reboot.publish(Float32(data=2.0))
        self.check_state("reboot: silent state_loss first", "LATCHED damp state_loss silent")
        self.check("armed reboot: RESTARTED, gap ~2 s", *self.gap("RESTARTED", 1.8, 2.5))
        self.check_state("armed reboot latches robot_restart", "LATCHED damp robot_restart silent")
        count = self.count
        self.spin(1.0)
        self.check("no /lowcmd to the rebooted robot", self.count == count, f"{self.count - count} frames")
        self.refused("estop_reset refused after a restart", self.client.estop_reset())
        self.refused("start_control refused after a restart", self.start(0.123))
        self.refused("ready refused after a restart", self.client._call(self.ready, Trigger.Request(), 30.0))

        # Without the motion service the restart still refuses: only a new bridge clears it.
        self.release.publish(Bool(data=True))
        self.spin(1.5)
        self.refused("estop_reset refused once /lowcmd is free", self.client.estop_reset())
        self.refused("start_control refused once /lowcmd is free", self.start(0.123))
        self.check("still RESTARTED", (self.link or "").startswith("RESTARTED"), self.link)
        self.check("nothing sent all along", self.count == count, f"{self.count - count} frames")

    def disarmed(self):
        if not self.up():
            return
        count = self.count
        self.reboot.publish(Float32(data=1.0))
        self.check("disarmed reboot: RESTARTED, gap ~1 s", *self.gap("RESTARTED", 0.8, 1.5))
        self.check_state("disarmed reboot latches nothing", "DISARMED")
        self.refused("start_control refused after a restart", self.start(0.123))
        self.refused("ready refused after a restart", self.client._call(self.ready, Trigger.Request(), 30.0))
        self.spin(0.5)
        self.check("nothing sent", self.count == count, f"{self.count - count} frames")
        self.release.publish(Bool(data=True))  # the next bridge's /lowcmd wait
        self.spin(0.5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", choices=("armed", "disarmed"))
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("link_driver")
    driver = Driver(node)
    try:
        getattr(driver, args.run)()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if driver.failures:
        print("\n{} check(s) failed: {}".format(len(driver.failures), ", ".join(driver.failures)))
        sys.exit(1)
    print("\nall checks passed")


if __name__ == "__main__":
    main()
