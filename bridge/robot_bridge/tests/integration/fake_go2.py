"""Fake Go2: unitree_go /lowstate at 500 Hz with the remote's header; reads /lowcmd, never sends it.

Test input: /fake_go2/keys (UInt16, the remote's key mask, as fake_g1). Out: every CHECK_EVERY-th
/lowcmd's frame fields and its CRC check on /fake_go2/frame (String, JSON), for drive_go2.py.
"""

import json
import struct

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt16
from unitree_go.msg import LowCmd, LowState

NUM_MOTORS = 12
STATE_PERIOD = 0.002
CHECK_EVERY = 25  # the CRC in Python takes ~10 ms: every frame would stall /lowstate
# unitree_sdk2py utils/crc.py: LowCmd_ packed little-endian, 4-byte aligned, 812 bytes.
LOWCMD_FMT = "<4B4IH2x" + "B3x5f3I" * 20 + "4B" + "55Bx2I"


def crc32(words):
    """Unitree's CRC over 32-bit words (crc32_core)."""
    crc = 0xFFFFFFFF
    for word in words:
        bit = 1 << 31
        for _ in range(32):
            crc = ((crc << 1) & 0xFFFFFFFF) ^ (0x04C11DB7 if crc & 0x80000000 else 0)
            if word & bit:
                crc ^= 0x04C11DB7
            bit >>= 1
    return crc


def lowcmd_crc(msg):
    """The CRC the robot computes over a received LowCmd."""
    data = [*msg.head, msg.level_flag, msg.frame_reserve, *msg.sn, *msg.version, msg.bandwidth]
    for m in msg.motor_cmd:
        data += [m.mode, m.q, m.dq, m.tau, m.kp, m.kd, *m.reserve]
    data += [msg.bms_cmd.off, *msg.bms_cmd.reserve, *msg.wireless_remote, *msg.led, *msg.fan,
             msg.gpio, msg.reserve, msg.crc]  # fmt: skip
    packed = struct.pack(LOWCMD_FMT, *data)
    return crc32(struct.unpack(f"<{len(packed) // 4 - 1}I", packed[:-4]))


class FakeGo2(Node):
    def __init__(self):
        super().__init__("fake_go2")
        self.state = LowState()
        self.state.imu_state.quaternion = [1.0, 0.0, 0.0, 0.0]
        self.state.wireless_remote[0] = 0x55  # the remote's header, as the G1 sends it
        self.state.wireless_remote[1] = 0x51
        self.q = [0.0, 1.26, -2.8] * 4  # lying, as measured on the real Go2
        self.publisher = self.create_publisher(LowState, "/lowstate", 10)
        self.frames = self.create_publisher(String, "/fake_go2/frame", 10)
        self.create_subscription(LowCmd, "/lowcmd", self._on_cmd, 10)
        self.create_subscription(UInt16, "/fake_go2/keys", self._on_keys, 10)
        self.t = 0
        self.received = 0
        self.create_timer(STATE_PERIOD, self._tick)

    def _tick(self):
        self.t += 2
        for i in range(NUM_MOTORS):
            self.state.motor_state[i].q = self.q[i]
        self.state.tick = self.t
        self.publisher.publish(self.state)

    def _on_keys(self, msg):
        self.state.wireless_remote[2] = msg.data & 0xFF
        self.state.wireless_remote[3] = msg.data >> 8

    def _on_cmd(self, msg):
        for i in range(NUM_MOTORS):
            if msg.motor_cmd[i].kp > 0:
                self.q[i] = msg.motor_cmd[i].q  # perfect tracking
        self.received += 1
        if self.received % CHECK_EVERY:
            return
        frame = {
            "head": [int(b) for b in msg.head],  # numpy uint8: json takes no numpy type
            "level_flag": int(msg.level_flag),
            "crc_ok": bool(lowcmd_crc(msg) == msg.crc),
            "modes": [int(m.mode) for m in msg.motor_cmd],
            "spare_q": [m.q for m in msg.motor_cmd[NUM_MOTORS:]],
            "kp": [m.kp for m in msg.motor_cmd[:NUM_MOTORS]],
            "kd": [m.kd for m in msg.motor_cmd[:NUM_MOTORS]],
        }
        self.frames.publish(String(data=json.dumps(frame)))


def main():
    rclpy.init()
    node = FakeGo2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
