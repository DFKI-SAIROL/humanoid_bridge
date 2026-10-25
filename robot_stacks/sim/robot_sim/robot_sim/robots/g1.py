"""The G1 in sim: mjlab's g1.xml (or unitree's with Dex3); unitree_hg on /lowstate, /lowcmd, /dex3/*."""

import os
import pathlib

import numpy as np

from robot_sim.physics import RobotSpec
from robot_sim.robots import unitree

DEFAULT_MJCF = pathlib.Path(
    os.environ.get("G1_MJCF", "~/.local/share/robot_web_commander/g1_mjlab/g1.xml")
).expanduser()

# Bridge and LowState motor order (G1_config.yaml joint_names); mjlab's names.
JOINT_NAMES = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)  # fmt: skip
NUM_JOINT = len(JOINT_NAMES)

# Dex3 motor order (G1_config.yaml {left,right}_hand_joint_names); unitree g1_29dof_with_hand.xml names.
HAND_SIDES = ("left", "right")
HAND_JOINTS = {
    side: tuple(f"{side}_hand_{finger}_joint"
                for finger in ("thumb_0", "thumb_1", "thumb_2", "middle_0", "middle_1", "index_0", "index_1"))
    for side in HAND_SIDES
}  # fmt: skip
NUM_HAND_JOINT = 7
HAND_EFFORT = 1.4  # Nm, the MJCF's actuatorfrcrange for a finger without one
HAND_DAMPING = 0.01  # Nm s/rad: a slack finger settles instead of ringing
HAND_ARMATURE = 0.002  # kg m^2, reflected rotor inertia (estimate): without it kd >= 0.1 rings at 2 ms

# (armature kg m^2, effort limit Nm) per motor, from mjlab g1_constants.py.
_5020 = (0.003609725, 25.0)
_7520_14 = (0.010177520, 88.0)
_7520_22 = (0.025101925, 139.0)
_4010 = (0.00425, 5.0)
_2X5020 = (2 * _5020[0], 2 * _5020[1])  # ankles and waist roll/pitch: two 5020 in parallel
MOTORS = {
    "hip_pitch": _7520_14, "hip_roll": _7520_22, "hip_yaw": _7520_14, "knee": _7520_22,
    "ankle_pitch": _2X5020, "ankle_roll": _2X5020,
    "waist_yaw": _7520_14, "waist_roll": _2X5020, "waist_pitch": _2X5020,
    "shoulder_pitch": _5020, "shoulder_roll": _5020, "shoulder_yaw": _5020, "elbow": _5020,
    "wrist_roll": _5020, "wrist_pitch": _4010, "wrist_yaw": _4010,
}  # fmt: skip

# Gearbox friction, as GEAR's g1 MJCF; without it limp legs spin the hip yaws to their limit.
FRICTIONLOSS = 0.2  # Nm
DAMPING = 0.05  # Nm s/rad

# Unitree g1_29dof.urdf d435_joint, on torso_link.
D435_POS = (0.0576235, 0.01753, 0.41987)
D435_PITCH = 0.8307767239493009
CAMERA = "d435"

MODE_MACHINE = 5  # as fake_g1; the bridge logs "G1 type:" once it is non-zero
HAND_PERIOD_S = 0.01  # 100 Hz, as the Dex3 and fake_dex3; the bridge aborts on state older than 0.2 s


def motor(joint_name):
    """(armature, effort) for a JOINT_NAMES entry."""
    key = joint_name.removesuffix("_joint").removeprefix("left_").removeprefix("right_")
    return MOTORS[key]


def _quat_y(angle):
    return [np.cos(angle / 2), 0.0, np.sin(angle / 2), 0.0]


def prepare(spec):
    """Armature and friction per motor, finger damping, the d435 camera on torso_link."""
    for name in JOINT_NAMES:
        joint = spec.joint(name)
        joint.armature = motor(name)[0]
        joint.frictionloss, joint.damping = FRICTIONLOSS, DAMPING
    names = {joint.name for joint in spec.joints}
    for name in (n for side in HAND_SIDES for n in HAND_JOINTS[side] if n in names):
        joint = spec.joint(name)
        joint.damping, joint.armature = HAND_DAMPING, HAND_ARMATURE
    # URDF frame is x forward; a MuJoCo camera looks along -z with y up.
    mount = spec.body("torso_link").add_body(
        name="d435_link", pos=list(D435_POS), quat=_quat_y(D435_PITCH)
    )
    mount.add_camera(name=CAMERA, xyaxes=[0, -1, 0, 0, 0, 1], fovy=58.0)


SPEC = RobotSpec(
    name="G1",
    joint_names=JOINT_NAMES,
    effort=lambda name: motor(name)[1],
    base="pelvis",
    imu_site="imu_in_pelvis",
    camera=CAMERA,
    default_mjcf=DEFAULT_MJCF,
    prepare=prepare,
    hand_joints=HAND_JOINTS,
    hand_effort=HAND_EFFORT,
)


class Wire:
    """unitree_hg: LowState at the physics rate, LowCmd in, Dex3 at 100 Hz, the motion service."""

    def __init__(self, node, sim, lock):
        from unitree_hg.msg import HandCmd, HandState, LowCmd, LowState, MotorState

        self.node = node
        self.sim = sim
        self.publisher = node.create_publisher(LowState, "/lowstate", 10)
        node.create_subscription(LowCmd, "/lowcmd", self._on_cmd, 10)
        self.motion = unitree.MotionService(
            node, LowCmd, "/lowcmd", LowCmd(mode_machine=MODE_MACHINE), lock  # every motor mode 0
        )
        self.state = LowState()
        self.state.mode_machine = MODE_MACHINE
        unitree.set_remote(self.state.wireless_remote, True)

        self.hand_publishers = {}
        self.hand_states = {}
        for side in HAND_SIDES if sim.hands else ():
            self.hand_publishers[side] = node.create_publisher(HandState, f"/dex3/{side}/state", 10)
            self.hand_states[side] = HandState(motor_state=[MotorState() for _ in range(NUM_HAND_JOINT)])
            node.create_subscription(
                HandCmd, f"/dex3/{side}/cmd", lambda msg, s=side: self._on_hand_cmd(s, msg), 10
            )
        self.hand_period = HAND_PERIOD_S if sim.hands else None
        self.describe = "Dex3 on /dex3/*" if sim.hands else "no Dex3"

    def _on_cmd(self, msg):
        cmd = self.sim.command()
        for i in range(NUM_JOINT):
            m = msg.motor_cmd[i]
            cmd.mode[i], cmd.q[i], cmd.dq[i] = m.mode, m.q, m.dq
            cmd.kp[i], cmd.kd[i], cmd.tau[i] = m.kp, m.kd, m.tau
        self.node.take_cmd(cmd)

    def _on_hand_cmd(self, side, msg):
        cmd = self.sim.hand_command(side)
        for i in range(min(len(msg.motor_cmd), NUM_HAND_JOINT)):  # unbounded MotorCmd[] on the wire
            m = msg.motor_cmd[i]
            cmd.q[i], cmd.dq[i], cmd.kp[i], cmd.kd[i], cmd.tau[i] = m.q, m.dq, m.kp, m.kd, m.tau
        self.node.take_hand_cmd(side, cmd)

    def set_remote(self, connected):
        unitree.set_remote(self.state.wireless_remote, connected)

    def press(self, names):
        """Hold these buttons (names as the bridge YAML's combos); [] releases."""
        unitree.set_keys(self.state.wireless_remote, unitree.key_mask(names))

    def boot(self):
        """End of a reboot, lock held."""
        self.motion.boot()

    def tick(self, now):
        """Lock held, link up: the motion service's frame when due."""
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
        for side, s in states.items():
            msg = self.hand_states[side]
            for i, ms in enumerate(msg.motor_state):
                ms.q, ms.dq, ms.tau_est = float(s["q"][i]), float(s["dq"][i]), float(s["tau_est"][i])
            self.hand_publishers[side].publish(msg)
