"""The Booster T1 in sim: ncbf's or Booster's model; booster_interface on /low_state, /joint_ctrl.

Its own side: booster_rpc_service (the mode switch; /joint_ctrl drives the motors in custom mode
only, other modes damp), and the gamepad on /remote_controller_state at 50 Hz.
"""

import json
import os
import pathlib

import numpy as np

from robot_sim.physics import RobotSpec

DEFAULT_MJCF = pathlib.Path(
    os.environ.get("T1_MJCF", "~/.local/share/robot_web_commander/t1_ncbf/t1.xml")
).expanduser()

# Bridge and LowState motor order (T1_config.yaml joint_names), as booster_assets T1_23dof.xml
# names them; ncbf's plane.xml names come second and are renamed on load.
JOINTS = (  # (Booster name, ncbf name, effort Nm from ncbf's actuatorfrcrange)
    ("aahead_yaw_joint", "AAHead_yaw", 7.0),
    ("aahead_pitch_joint", "Head_pitch", 7.0),
    ("left_shoulder_pitch_joint", "Left_Shoulder_Pitch", 38.3),
    ("left_shoulder_roll_joint", "Left_Shoulder_Roll", 38.3),
    ("left_elbow_pitch_joint", "Left_Elbow_Pitch", 38.3),
    ("left_elbow_yaw_joint", "Left_Elbow_Yaw", 38.3),
    ("right_shoulder_pitch_joint", "Right_Shoulder_Pitch", 38.3),
    ("right_shoulder_roll_joint", "Right_Shoulder_Roll", 38.3),
    ("right_elbow_pitch_joint", "Right_Elbow_Pitch", 38.3),
    ("right_elbow_yaw_joint", "Right_Elbow_Yaw", 38.3),
    ("waist_yaw_joint", "Waist", 68.0),
    ("left_hip_pitch_joint", "Left_Hip_Pitch", 96.0),
    ("left_hip_roll_joint", "Left_Hip_Roll", 68.0),
    ("left_hip_yaw_joint", "Left_Hip_Yaw", 68.0),
    ("left_knee_pitch_joint", "Left_Knee_Pitch", 130.0),
    ("left_ankle_pitch_joint", "Left_Ankle_Pitch", 76.0),
    ("left_ankle_roll_joint", "Left_Ankle_Roll", 76.0),
    ("right_hip_pitch_joint", "Right_Hip_Pitch", 96.0),
    ("right_hip_roll_joint", "Right_Hip_Roll", 68.0),
    ("right_hip_yaw_joint", "Right_Hip_Yaw", 68.0),
    ("right_knee_pitch_joint", "Right_Knee_Pitch", 130.0),
    ("right_ankle_pitch_joint", "Right_Ankle_Pitch", 76.0),
    ("right_ankle_roll_joint", "Right_Ankle_Roll", 76.0),
)
JOINT_NAMES = tuple(booster for booster, _, _ in JOINTS)
NUM_JOINT = len(JOINT_NAMES)
EFFORT = {booster: effort for booster, _, effort in JOINTS}
# ncbf's body names -> Booster's, for the base and the camera mount.
BODIES = {"Trunk": "trunk", "H2": "aahead_pitch_link"}

# booster_assets T1_23dof.xml head_realsense_rgb_link, an optical frame (z forward, y down) on
# aahead_pitch_link. A MuJoCo camera looks along -z with y up: x kept, y flipped.
CAMERA_POS = (0.063961, -0.0115, 0.108406)
CAMERA_QUAT = (0.46390386, -0.53366035, 0.53366018, -0.46390371)
CAMERA = "head_rgb"

# booster_mode.hpp: RobotMode and the change-mode API.
DAMPING, PREPARE, WALKING, CUSTOM = 0, 1, 2, 3
CHANGE_MODE = 2000
DAMP_KD = 2.0  # Nm s/rad: a joint outside custom mode (damping; prepare and walking too, here)
PAD_PERIOD_S = 0.02
# Button names (the bridge's estop.remote_keys t1) -> RemoteControllerState fields.
PAD = {"A": "a", "B": "b", "X": "x", "Y": "y", "LB": "lb", "RB": "rb", "LT": "lt", "RT": "rt",
       "BACK": "back", "START": "start", "LS": "ls", "RS": "rs",
       "UP": "hat_u", "DOWN": "hat_d", "LEFT": "hat_l", "RIGHT": "hat_r"}  # fmt: skip


def prepare(spec):
    """ncbf's names to Booster's, joint friction off, the head camera on aahead_pitch_link."""
    joints = {joint.name: joint for joint in spec.joints}
    for booster, ncbf, _ in JOINTS:
        if ncbf in joints:
            joints[ncbf].name = booster
    for actuator in spec.actuators:  # ncbf's motors name their joint
        renamed = {ncbf: booster for booster, ncbf, _ in JOINTS}
        actuator.target = renamed.get(actuator.target, actuator.target)
    for body in spec.bodies:
        body.name = BODIES.get(body.name, body.name)
    mount = spec.body("aahead_pitch_link").add_body(
        name="head_rgb_link", pos=list(CAMERA_POS), quat=list(CAMERA_QUAT)
    )
    mount.add_camera(name=CAMERA, xyaxes=[1, 0, 0, 0, -1, 0], fovy=58.0)


SPEC = RobotSpec(
    name="T1",
    joint_names=JOINT_NAMES,
    effort=EFFORT.__getitem__,
    base="trunk",
    imu_site="imu",
    camera=CAMERA,
    default_mjcf=DEFAULT_MJCF,
    prepare=prepare,
)


def mode_request(body):
    """The mode a change-mode body asks for; None when it is not one."""
    try:
        mode = json.loads(body or "{}").get("mode")
    except (ValueError, AttributeError):
        return None
    return mode if mode in (DAMPING, PREPARE, WALKING, CUSTOM) else None


class Wire:
    """booster_interface: LowState at the physics rate, LowCmd, the RPC mode switch, the gamepad."""

    def __init__(self, node, sim, lock):
        from booster_interface.msg import LowCmd, LowState, MotorState, RemoteControllerState
        from booster_interface.srv import RpcService

        self.node = node
        self.sim = sim
        self.lock = lock
        self.mode = DAMPING  # as the T1 boots; guarded by lock
        self.publisher = node.create_publisher(LowState, "/low_state", 10)
        node.create_subscription(LowCmd, "/joint_ctrl", self._on_cmd, 10)
        node.create_service(RpcService, "booster_rpc_service", self._on_rpc)
        pad_topic = "/remote_controller_state"
        self.pad_publisher = node.create_publisher(RemoteControllerState, pad_topic, 10)
        self.Pad = RemoteControllerState
        self.pad = RemoteControllerState()
        self.remote = True
        self.pad_due = 0.0
        self.state = LowState(motor_state_serial=[MotorState() for _ in range(NUM_JOINT)])
        damp = sim.command()
        damp.mode[:], damp.kd[:] = 1, DAMP_KD
        self.damp = damp
        self.hand_period = None
        self.describe = "booster_interface on /low_state, /joint_ctrl; mode damping"

    def _on_cmd(self, msg):
        cmd = self.sim.command()
        for i in range(min(len(msg.motor_cmd), NUM_JOINT)):
            m = msg.motor_cmd[i]
            cmd.mode[i] = 1
            cmd.q[i], cmd.dq[i], cmd.tau[i] = m.q, m.dq, m.tau
            cmd.kp[i], cmd.kd[i] = m.kp, m.kd
        self.node.take_cmd(cmd)

    def _on_rpc(self, request, response):
        mode = mode_request(request.msg.body) if request.msg.api_id == CHANGE_MODE else None
        if mode is None:
            response.msg.status = 100  # not a mode this fake knows
            return response
        with self.lock:
            self.mode = mode
        self.node.get_logger().info(f"booster_rpc_service: mode {mode}")
        response.msg.status = 0
        return response

    def filter(self, cmd):
        """Lock held: /joint_ctrl drives the motors in custom mode only; any other mode damps."""
        return cmd if self.mode == CUSTOM else self.damp

    def set_remote(self, connected):
        self.remote = connected

    def press(self, names):
        """Hold these buttons (the bridge's T1 names: RB, RT, LB ...); [] releases."""
        unknown = [name for name in names if name.upper() not in PAD]
        if unknown:
            raise ValueError(f"button {unknown[0]!r}: expected one of {tuple(PAD)}")
        held = {PAD[name.upper()] for name in names}
        self.pad = self.Pad(**{field: field in held for field in PAD.values()})

    def boot(self):
        """End of a reboot, lock held: the T1 comes up damping."""
        self.mode = DAMPING

    def tick(self, now):
        """Lock held, link up: the gamepad at PAD_PERIOD_S while it is on."""
        if self.remote and now >= self.pad_due:
            self.pad_due = now + PAD_PERIOD_S
            self.pad_publisher.publish(self.pad)

    def publish(self, s):
        for i in range(NUM_JOINT):
            ms = self.state.motor_state_serial[i]
            ms.q, ms.dq = float(s["q"][i]), float(s["dq"][i])
            ms.ddq, ms.tau_est = float(s["ddq"][i]), float(s["tau_est"][i])
        imu = self.state.imu_state
        imu.rpy = s["rpy"].astype(np.float32).tolist()
        imu.gyro = s["gyroscope"].astype(np.float32).tolist()
        imu.acc = s["accelerometer"].astype(np.float32).tolist()
        self.publisher.publish(self.state)

    def publish_hands(self, states):
        pass  # no hands
