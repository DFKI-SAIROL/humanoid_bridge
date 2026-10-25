"""The Unitree Go2 in sim: ncbf's or mujoco_menagerie's model; unitree_go on /lowstate, /lowcmd.

Its own side: the CRC on every /lowcmd (a wrong one is dropped, as the robot does), the sport
service's StandDown and the motion switcher (unitree_api), the Unitree remote on LowState. It starts
lying on the floor, no gantry.
"""

import os
import pathlib
import struct

import numpy as np

from robot_sim.physics import RobotSpec
from robot_sim.robots import unitree

DEFAULT_MJCF = pathlib.Path(
    os.environ.get("GO2_MJCF", "~/.local/share/robot_web_commander/go2_ncbf/go2.xml")
).expanduser()

# Bridge and LowState motor order (Go2_config.yaml joint_names; unitree_go motor index).
LEGS = ("FR", "FL", "RR", "RL")
JOINT_NAMES = tuple(f"{leg}_{part}_joint" for leg in LEGS for part in ("hip", "thigh", "calf"))
NUM_JOINT = len(JOINT_NAMES)
EFFORT = {"hip": 23.7, "thigh": 23.7, "calf": 45.43}  # Nm, the go2 URDF / ncbf's forcerange
# Lying, as measured on the real Go2 (the ncbf deploy's lying_joint_positions).
LYING_Q = (-0.046, 1.265, -2.797, 0.034, 1.255, -2.785, -0.343, 1.278, -2.803, 0.343, 1.278, -2.811)

# ncbf's go2 ego_camera on the trunk; menagerie's model has none and gets the same.
CAMERA = "ego_camera"
CAMERA_POS = (0.312, 0.0, 0.045)
CAMERA_QUAT = (-0.5, -0.5, 0.5, 0.5)

NUM_SLOTS = 20  # unitree_go LowCmd.motor_cmd
BOOT_MODE = "normal"  # the mode a booted Go2's sport service holds /lowcmd in
SPORT_REQUEST = "/api/sport/request"
SPORT_RESPONSE = "/api/sport/response"
# unitree_sdk2py utils/crc.py: LowCmd_ packed little-endian, 4-byte aligned, 812 bytes.
LOWCMD_FMT = "<4B4IH2x" + "B3x5f3I" * 20 + "4B" + "55Bx2I"


def _crc_table():
    table = []
    for byte in range(256):
        crc = byte << 24
        for _ in range(8):
            crc = ((crc << 1) ^ 0x04C11DB7) if crc & 0x80000000 else (crc << 1)
            crc &= 0xFFFFFFFF
        table.append(crc)
    return table


_TABLE = _crc_table()


def crc32_words(words):
    """Unitree's crc32_core: MSB first over 32-bit words = CRC-32/MPEG-2 over big-endian bytes."""
    crc = 0xFFFFFFFF
    for byte in struct.pack(f">{len(words)}I", *words):
        crc = ((crc << 8) & 0xFFFFFFFF) ^ _TABLE[(crc >> 24) ^ byte]
    return crc


def lowcmd_crc(msg):
    """The CRC the Go2 computes over a received LowCmd (all but the last word)."""
    data = [*msg.head, msg.level_flag, msg.frame_reserve, *msg.sn, *msg.version, msg.bandwidth]
    for m in msg.motor_cmd:
        data += [m.mode, m.q, m.dq, m.tau, m.kp, m.kd, *m.reserve]
    data += [msg.bms_cmd.off, *msg.bms_cmd.reserve, *msg.wireless_remote, *msg.led, *msg.fan,
             msg.gpio, msg.reserve, msg.crc]  # fmt: skip
    packed = struct.pack(LOWCMD_FMT, *data)
    return crc32_words(struct.unpack(f"<{len(packed) // 4 - 1}I", packed[:-4]))


def prepare(spec):
    """Base named trunk (menagerie: base), ncbf's position actuators off, the ego_camera."""
    for body in spec.bodies:
        if body.name == "base":
            body.name = "trunk"
    for actuator in spec.actuators:  # ncbf's <position kp=20>: at ctrl 0 it pulls every joint to 0
        actuator.gainprm[0] = 0.0
        actuator.biasprm[:3] = 0.0
    if CAMERA not in {camera.name for camera in spec.cameras}:
        spec.body("trunk").add_camera(name=CAMERA, pos=list(CAMERA_POS), quat=list(CAMERA_QUAT))


SPEC = RobotSpec(
    name="Go2",
    joint_names=JOINT_NAMES,
    effort=lambda name: EFFORT[name.split("_")[1]],
    base="trunk",
    imu_site="imu",
    camera=CAMERA,
    default_mjcf=DEFAULT_MJCF,
    prepare=prepare,
    start_q=LYING_Q,
    gantry=False,
)


class Wire:
    """unitree_go: LowState at the physics rate, LowCmd (CRC checked), sport, motion switcher."""

    def __init__(self, node, sim, lock):
        from unitree_api.msg import Request, Response
        from unitree_go.msg import LowCmd, LowState

        self.node = node
        self.sim = sim
        self.publisher = node.create_publisher(LowState, "/lowstate", 10)
        node.create_subscription(LowCmd, "/lowcmd", self._on_cmd, 10)
        self.motion = unitree.MotionService(
            node, LowCmd, "/lowcmd", LowCmd(), lock, boot_mode=BOOT_MODE
        )
        node.create_subscription(Request, SPORT_REQUEST, self._on_sport, 10)
        self.sport_response = node.create_publisher(Response, SPORT_RESPONSE, 10)
        self.Response = Response
        self.state = LowState()
        unitree.set_remote(self.state.wireless_remote, True)
        self.bad_crc = 0
        self.hand_period = None
        self.describe = "unitree_go on /lowstate, /lowcmd (CRC checked); starts lying"

    def _on_cmd(self, msg):
        if lowcmd_crc(msg) != msg.crc:
            self.bad_crc += 1
            if self.bad_crc in (1, 10, 100) or self.bad_crc % 1000 == 0:
                log = self.node.get_logger()
                log.error(f"/lowcmd with a wrong CRC dropped ({self.bad_crc} so far)")
            return
        cmd = self.sim.command()
        for i in range(NUM_JOINT):
            m = msg.motor_cmd[i]
            cmd.mode[i], cmd.q[i], cmd.dq[i] = m.mode, m.q, m.dq
            cmd.kp[i], cmd.kd[i], cmd.tau[i] = m.kp, m.kd, m.tau
        self.node.take_cmd(cmd)

    def _on_sport(self, msg):
        """Every sport call (StandDown, Damp ...) answers ok; the limp sim already lies."""
        response = self.Response()
        response.header.identity = msg.header.identity
        response.header.status.code = 0
        self.sport_response.publish(response)
        self.node.get_logger().info(f"sport: api {msg.header.identity.api_id} -> 0")

    def filter(self, cmd):
        """Lock held: the Go2 follows /lowcmd in every mode."""
        return cmd

    def set_remote(self, connected):
        unitree.set_remote(self.state.wireless_remote, connected)

    def press(self, names):
        """Hold these buttons (names as the bridge YAML's combos); [] releases."""
        unitree.set_keys(self.state.wireless_remote, unitree.key_mask(names))

    def boot(self):
        """End of a reboot, lock held: the sport service holds /lowcmd again."""
        self.motion.boot()

    def tick(self, now):
        """Lock held, link up: the sport service's frame when due."""
        self.motion.send(now)

    def publish(self, s):
        for i in range(NUM_JOINT):
            ms = self.state.motor_state[i]
            ms.q, ms.dq = float(s["q"][i]), float(s["dq"][i])
            ms.ddq, ms.tau_est = float(s["ddq"][i]), float(s["tau_est"][i])
        imu = self.state.imu_state
        imu.quaternion = s["quaternion"].astype(np.float32).tolist()
        imu.rpy = s["rpy"].astype(np.float32).tolist()
        imu.gyroscope = s["gyroscope"].astype(np.float32).tolist()
        imu.accelerometer = s["accelerometer"].astype(np.float32).tolist()
        self.state.tick = int(s["time"] * 1000)
        self.publisher.publish(self.state)

    def publish_hands(self, states):
        pass  # no hands
