"""Fake G1: publishes /lowstate at 500 Hz and records /lowcmd. Never publishes /lowcmd.

Test inputs: /fake_g1/keys (UInt16, remote key mask), /fake_g1/roll (Float32, rad),
/fake_g1/remote (Bool, remote header on/off), /fake_g1/pause (Bool, stop /lowstate).
Bad robot data, joint 0 / roll: /fake_g1/nan_q (Bool), /fake_g1/dq (Float32, rad/s),
/fake_g1/nan_imu (Bool). /fake_g1/freeze (Bool): stop tracking, q holds (a lagging robot).
/fake_g1/pause is a link drop: LowState.tick moves on. /fake_g1/reboot (Float32, s): no /lowstate
for s, then tick from 0 and a motion service holding /lowcmd (never sends); /fake_g1/release (Bool) drops it.
"""

import argparse
import json
import math
import signal
import struct
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, UInt16
from unitree_hg.msg import LowCmd, LowState

NUM_JOINT = 29
STATE_PERIOD = 0.002  # the bridge aborts on /lowstate older than 0.2 s
MODE_MACHINE = 5  # G1 type; the bridge copies it into every /lowcmd
# Values drive_robot_state.py finds again on /robot_state.
GYRO = [0.1, 0.2, 0.3]
ACC = [0.0, 0.0, 9.81]
STICKS = {"lx": 0.25, "rx": -0.5, "ry": 0.75, "ly": -0.125}  # wireless_remote at 4, 8, 12, 20


def tau_est(i):
    return 0.01 * i


class FakeG1(Node):
    def __init__(self, out_path, start_q=0.0):
        super().__init__("fake_g1")
        self.out_path = out_path
        self.publisher = self.create_publisher(LowState, "/lowstate", 10)
        self.create_subscription(LowCmd, "/lowcmd", self._on_cmd, 10)
        self.create_subscription(UInt16, "/fake_g1/keys", self._on_keys, 10)
        self.create_subscription(Float32, "/fake_g1/roll", self._on_roll, 10)
        self.create_subscription(Bool, "/fake_g1/remote", self._on_remote, 10)
        self.create_subscription(Bool, "/fake_g1/pause", self._on_pause, 10)
        self.create_subscription(Bool, "/fake_g1/nan_q", self._on_nan_q, 10)
        self.create_subscription(Float32, "/fake_g1/dq", self._on_dq, 10)
        self.create_subscription(Bool, "/fake_g1/nan_imu", self._on_nan_imu, 10)
        self.create_subscription(Bool, "/fake_g1/freeze", self._on_freeze, 10)
        self.create_subscription(Float32, "/fake_g1/reboot", self._on_reboot, 10)
        self.create_subscription(Bool, "/fake_g1/release", self._on_release, 10)
        self.boot_until = None  # monotonic time the reboot ends
        self.motion_service = None
        self.frozen = False
        self.paused = False
        self.nan_q = False
        self.dq0 = 0.0
        self.roll = 0.0
        self.nan_imu = False

        self.state = LowState()
        self.state.mode_machine = MODE_MACHINE
        self.state.imu_state.rpy = [0.0, 0.0, 0.0]  # level: over 1 rad aborts control
        self.state.imu_state.quaternion = [1.0, 0.0, 0.0, 0.0]
        self.state.wireless_remote[0] = 0x55  # header bytes as the real G1 sends them
        self.state.wireless_remote[1] = 0x51
        self.state.imu_state.gyroscope = GYRO
        self.state.imu_state.accelerometer = ACC
        for offset, name in ((4, "lx"), (8, "rx"), (12, "ry"), (20, "ly")):
            self.state.wireless_remote[offset : offset + 4] = list(struct.pack("<f", STICKS[name]))

        self.q = [float(start_q)] * NUM_JOINT
        self.samples = []
        self.t0 = time.monotonic()
        self.booted = self.t0  # LowState.tick counts from here
        self.create_timer(STATE_PERIOD, self._tick)

    def _tick(self):
        if self.boot_until is not None:
            if time.monotonic() < self.boot_until:
                return
            self.boot_until = None
            self.booted = time.monotonic()
            if self.motion_service is None:
                self.motion_service = self.create_publisher(LowCmd, "/lowcmd", 10)
            print("fake_g1: rebooted, motion service holds /lowcmd", flush=True)
        if self.paused:
            return
        for i in range(NUM_JOINT):
            self.state.motor_state[i].q = self.q[i]
            self.state.motor_state[i].dq = 0.0
            self.state.motor_state[i].tau_est = tau_est(i)
        if self.nan_q:
            self.state.motor_state[0].q = math.nan
        self.state.motor_state[0].dq = self.dq0
        self.state.imu_state.rpy = [math.nan if self.nan_imu else self.roll, 0.0, 0.0]
        self.state.tick = int((time.monotonic() - self.booted) * 1000)
        self.publisher.publish(self.state)

    def _on_keys(self, msg):
        self.state.wireless_remote[2] = msg.data & 0xFF  # key mask, little-endian
        self.state.wireless_remote[3] = msg.data >> 8

    def _on_roll(self, msg):
        self.roll = float(msg.data)

    def _on_nan_q(self, msg):
        self.nan_q = bool(msg.data)

    def _on_dq(self, msg):
        self.dq0 = float(msg.data)

    def _on_nan_imu(self, msg):
        self.nan_imu = bool(msg.data)

    def _on_freeze(self, msg):
        self.frozen = bool(msg.data)

    def _on_remote(self, msg):
        header = (0x55, 0x51) if msg.data else (0, 0)  # remote off: the G1 sends all 40 bytes 0
        self.state.wireless_remote[0], self.state.wireless_remote[1] = header

    def _on_pause(self, msg):
        self.paused = bool(msg.data)

    def _on_reboot(self, msg):
        self.boot_until = time.monotonic() + float(msg.data)

    def _on_release(self, msg):
        if msg.data and self.motion_service is not None:
            self.destroy_publisher(self.motion_service)
            self.motion_service = None
            print("fake_g1: motion service released /lowcmd", flush=True)

    def _on_cmd(self, msg):
        if not self.frozen:
            self.q = [msg.motor_cmd[i].q for i in range(NUM_JOINT)]  # perfect tracking
        self.samples.append(
            {
                "t": round(time.monotonic() - self.t0, 4),
                "mode_pr": int(msg.mode_pr),
                "mode_machine": int(msg.mode_machine),
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
    parser.add_argument("--start-q", type=float, default=0.0, help="every joint's q until /lowcmd arrives")
    args = parser.parse_args()

    rclpy.init()
    node = FakeG1(args.out, args.start_q)
    signal.signal(signal.SIGTERM, lambda *_: (node.dump(), sys.exit(0)))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.dump()


if __name__ == "__main__":
    main()
