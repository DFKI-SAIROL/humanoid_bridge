"""Drives /robot_link of a G1_bridge against robot_sim (G1): link drop and reboot, then a fresh bring-up.

Exit 1 on failure. run_sim_link_test.sh runs `restart`, restarts the bridge with g1-bringup.sh --sim,
then runs `rebrought`.
"""

import argparse
import sys

import rclpy
from drive_link import Driver as LinkDriver
from std_msgs.msg import Float32
from unitree_hg.msg import LowState

RAMP_SECONDS = 3.0  # `duration` 2.0 plus margin
NUM_MOTORS = 29


class Driver(LinkDriver):
    def __init__(self, node):
        super().__init__(node)
        self.link_down = node.create_publisher(Float32, "/robot_sim/link", 10)
        self.sim_reboot = node.create_publisher(Float32, "/robot_sim/reboot", 10)
        node.create_subscription(LowState, "/lowstate", self._on_low, 10)
        self.low = None

    def _on_low(self, msg):
        self.low = msg

    def _on_cmd(self, msg):
        # The fake motion service sends every motor at mode 0; the bridge sends mode 1.
        if any(m.mode for m in msg.motor_cmd[:NUM_MOTORS]):
            self.count += 1

    def up(self):
        ok = self.client.start_control_client.wait_for_service(timeout_sec=30.0)
        self.check("bridge up: /start_control", ok)
        inputs = (self.link_down, self.sim_reboot)
        matched = self.wait(lambda: all(p.get_subscription_count() for p in inputs), 5.0)
        self.check("robot_sim inputs matched", matched)
        self.check("/lowstate from the sim", self.wait(lambda: self.low is not None, 5.0))
        self.check_link("a new bridge starts OK", "OK")
        self.check_state("starts DISARMED", "DISARMED")
        return ok and matched and self.low is not None

    def arm(self):
        measured = [m.q for m in self.low.motor_state[:NUM_MOTORS]]
        reply = self.client.start_control(default_position=measured, timeout_sec=30.0)
        self.check("start_control from the measured pose", reply.success, reply.message)
        self.spin(RAMP_SECONDS)
        self.check_state("ARMED", "ARMED")

    def restart(self):
        if not self.up():
            return
        self.arm()

        # Link drop: sim time runs on, so the tick moves on.
        self.link_down.publish(Float32(data=3.0))
        self.check_state("link drop latches damp state_loss", "LATCHED damp state_loss")
        self.check_state("then silent", "LATCHED damp state_loss silent")
        self.check("link drop: BACK, gap ~3 s", *self.gap("BACK", 2.7, 3.8))
        self.check("BACK names no restart", (self.link or "").endswith(" no_restart"), self.link)
        reply = self.client.estop_reset()
        self.check("estop_reset after BACK", reply.success, reply.message)
        self.check_link("reset clears BACK -> OK", "OK")
        self.check_state("reset from silent -> DISARMED", "DISARMED")
        self.arm()

        # Reboot: the sim resets, its tick restarts, its motion service takes /lowcmd.
        self.sim_reboot.publish(Float32(data=2.0))
        self.check("reboot: RESTARTED, gap ~2 s", *self.gap("RESTARTED", 1.8, 2.8))
        self.check_state("reboot latches robot_restart", "LATCHED damp robot_restart silent")
        tick = self.low.tick if self.low else None
        self.check("the sim's tick restarted", tick is not None and tick < 5000, f"tick {tick} ms")
        count = self.count
        self.spin(1.0)
        self.check("motion service on /lowcmd", self.node.count_publishers("/lowcmd") >= 2,
                   f"{self.node.count_publishers('/lowcmd')} publishers")  # fmt: skip
        self.check("no bridge frame to the rebooted sim", self.count == count, f"{self.count - count} frames")
        self.refused("estop_reset refused after a restart", self.client.estop_reset())

    def rebrought(self):
        if not self.up():
            return
        publishers = self.node.count_publishers("/lowcmd")
        self.check("bring-up released the motion service", publishers == 1, f"{publishers} /lowcmd publishers")
        self.arm()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", choices=("restart", "rebrought"))
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("sim_link_driver")
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
