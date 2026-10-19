"""Fake G1: publishes /lowstate at 500 Hz and records /lowcmd. Never publishes /lowcmd.

Test inputs: /fake_g1/keys (UInt16, remote key mask), /fake_g1/roll (Float32, rad).
"""

import argparse
import json
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, UInt16
from unitree_hg.msg import LowCmd, LowState

NUM_JOINT = 29
STATE_PERIOD = 0.002  # the bridge aborts on /lowstate older than 0.2 s


class FakeG1(Node):
    def __init__(self, out_path):
        super().__init__("fake_g1")
        self.out_path = out_path
        self.publisher = self.create_publisher(LowState, "/lowstate", 10)
        self.create_subscription(LowCmd, "/lowcmd", self._on_cmd, 10)
        self.create_subscription(UInt16, "/fake_g1/keys", self._on_keys, 10)
        self.create_subscription(Float32, "/fake_g1/roll", self._on_roll, 10)

        self.state = LowState()
        self.state.mode_machine = 5
        self.state.imu_state.rpy = [0.0, 0.0, 0.0]  # level: over 1 rad aborts control
        self.state.imu_state.quaternion = [1.0, 0.0, 0.0, 0.0]
        self.state.wireless_remote[0] = 0x55  # header bytes as the real G1 sends them
        self.state.wireless_remote[1] = 0x51

        self.q = [0.0] * NUM_JOINT
        self.samples = []
        self.t0 = time.monotonic()
        self.create_timer(STATE_PERIOD, self._tick)

    def _tick(self):
        for i in range(NUM_JOINT):
            self.state.motor_state[i].q = self.q[i]
            self.state.motor_state[i].dq = 0.0
        self.state.tick = int((time.monotonic() - self.t0) * 1000)
        self.publisher.publish(self.state)

    def _on_keys(self, msg):
        self.state.wireless_remote[2] = msg.data & 0xFF  # key mask, little-endian
        self.state.wireless_remote[3] = msg.data >> 8

    def _on_roll(self, msg):
        self.state.imu_state.rpy = [float(msg.data), 0.0, 0.0]

    def _on_cmd(self, msg):
        self.q = [msg.motor_cmd[i].q for i in range(NUM_JOINT)]  # perfect tracking
        self.samples.append(
            {
                "t": round(time.monotonic() - self.t0, 4),
                "mode_pr": int(msg.mode_pr),
                "mode": [int(msg.motor_cmd[i].mode) for i in range(NUM_JOINT)],
                "q": [round(msg.motor_cmd[i].q, 6) for i in range(NUM_JOINT)],
                "kp": [round(msg.motor_cmd[i].kp, 3) for i in range(NUM_JOINT)],
                "kd": [round(msg.motor_cmd[i].kd, 3) for i in range(NUM_JOINT)],
            }
        )

    def dump(self):
        with open(self.out_path, "w") as handle:
            json.dump(self.samples, handle)
        print(f"fake_g1: wrote {len(self.samples)} /lowcmd samples", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("out", help="where to write the recorded /lowcmd samples")
    args = parser.parse_args()

    rclpy.init()
    node = FakeG1(args.out)
    signal.signal(signal.SIGTERM, lambda *_: (node.dump(), sys.exit(0)))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.dump()


if __name__ == "__main__":
    main()
