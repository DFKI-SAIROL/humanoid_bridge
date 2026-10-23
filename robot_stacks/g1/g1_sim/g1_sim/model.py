"""MuJoCo G1 on mjlab's g1.xml (or unitree's with Dex3 hands): PD from LowCmd and HandCmd. No ROS."""

import os
import pathlib

import mujoco
import numpy as np

DEFAULT_MJCF = pathlib.Path(
    os.environ.get("G1_MJCF", "~/.local/share/g1_web_commander/g1_mjlab/g1.xml")
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


def motor(joint_name):
    """(armature, effort) for a JOINT_NAMES entry."""
    key = joint_name.removesuffix("_joint").removeprefix("left_").removeprefix("right_")
    return MOTORS[key]


def build_spec(mjcf=DEFAULT_MJCF):
    """mjlab's G1 plus floor, IMU sensors, armature and the d435 camera."""
    mjcf = pathlib.Path(mjcf)
    if not mjcf.is_file():
        raise FileNotFoundError(
            f"{mjcf}: no G1 model. Run g1_web_commander/scripts/fetch_assets.sh, or set G1_MJCF."
        )
    spec = mujoco.MjSpec.from_file(str(mjcf))
    if not (mjcf.parent / spec.meshdir).is_dir() and (mjcf.parent / "meshes").is_dir():
        spec.meshdir = "meshes"  # fetch_assets.sh renames mjlab's assets/ to meshes/

    if "floor" not in {geom.name for geom in spec.geoms}:  # unitree's scene MJCFs bring their own
        spec.add_texture(name="floor", type=mujoco.mjtTexture.mjTEXTURE_2D,
                         builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER, width=300, height=300,
                         rgb1=[0.2, 0.3, 0.4], rgb2=[0.1, 0.2, 0.3])  # fmt: skip
        floor = spec.add_material(name="floor", texrepeat=[5, 5], texuniform=True)
        floor.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "floor"
        spec.worldbody.add_geom(
            name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[0, 0, 0.05], material="floor"
        )
        spec.worldbody.add_light(pos=[0, 0, 4], dir=[0, 0, -1], diffuse=[0.7, 0.7, 0.7])
    for name in JOINT_NAMES:
        joint = spec.joint(name)
        joint.armature = motor(name)[0]
        joint.frictionloss, joint.damping = FRICTIONLOSS, DAMPING
    names = {joint.name for joint in spec.joints}
    for name in (n for side in HAND_SIDES for n in HAND_JOINTS[side] if n in names):
        joint = spec.joint(name)
        joint.damping, joint.armature = HAND_DAMPING, HAND_ARMATURE

    spec.add_sensor(name="imu_quat", type=mujoco.mjtSensor.mjSENS_FRAMEQUAT,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname="imu_in_pelvis")  # fmt: skip
    spec.add_sensor(name="imu_gyro", type=mujoco.mjtSensor.mjSENS_GYRO,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname="imu_in_pelvis")  # fmt: skip
    spec.add_sensor(name="imu_acc", type=mujoco.mjtSensor.mjSENS_ACCELEROMETER,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname="imu_in_pelvis")  # fmt: skip

    # URDF frame is x forward; a MuJoCo camera looks along -z with y up.
    mount = spec.body("torso_link").add_body(
        name="d435_link", pos=list(D435_POS), quat=_quat_y(D435_PITCH)
    )
    mount.add_camera(name=CAMERA, xyaxes=[0, -1, 0, 0, 0, 1], fovy=58.0)
    return spec


def _quat_y(angle):
    return [np.cos(angle / 2), 0.0, np.sin(angle / 2), 0.0]


def rpy_from_quat(q):
    """Roll, pitch, yaw (ZYX) from w, x, y, z — what LowState.imu_state.rpy carries."""
    w, x, y, z = q
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


class Command:
    """One LowCmd's 29 motor targets; mode 0 is a disabled motor."""

    def __init__(self):
        self.mode = np.zeros(NUM_JOINT, dtype=np.int32)
        self.q = np.zeros(NUM_JOINT)
        self.dq = np.zeros(NUM_JOINT)
        self.kp = np.zeros(NUM_JOINT)
        self.kd = np.zeros(NUM_JOINT)
        self.tau = np.zeros(NUM_JOINT)


class HandCommand:
    """One HandCmd's 7 finger targets; zero gains and torque until the first one: slack."""

    def __init__(self):
        self.q = np.zeros(NUM_HAND_JOINT)
        self.dq = np.zeros(NUM_HAND_JOINT)
        self.kp = np.zeros(NUM_HAND_JOINT)
        self.kd = np.zeros(NUM_HAND_JOINT)
        self.tau = np.zeros(NUM_HAND_JOINT)


class G1Sim:
    """Steps the G1 with the motor PD law; a gantry holds the pelvis while it is on.

    `hands`: the MJCF has both Dex3 hands; then `step` also takes a HandCommand per side.
    """

    GANTRY_KP = (4000.0, 400.0)  # N/m, Nm/rad
    GANTRY_KD = (400.0, 40.0)

    def __init__(self, mjcf=DEFAULT_MJCF, clearance=0.05):
        self.model = build_spec(mjcf).compile()
        self.data = mujoco.MjData(self.model)
        m = self.model
        joints = [m.joint(n) for n in JOINT_NAMES]
        self.qadr = np.array([j.qposadr[0] for j in joints])
        self.vadr = np.array([j.dofadr[0] for j in joints])
        self.effort = np.array([motor(n)[1] for n in JOINT_NAMES])
        self.hands = all(
            mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) >= 0
            for side in HAND_SIDES for n in HAND_JOINTS[side]
        )  # fmt: skip
        self.hand_qadr, self.hand_vadr, self.hand_effort, self.hand_tau = {}, {}, {}, {}
        for side in HAND_SIDES if self.hands else ():
            fingers = [m.joint(n) for n in HAND_JOINTS[side]]
            self.hand_qadr[side] = np.array([j.qposadr[0] for j in fingers])
            self.hand_vadr[side] = np.array([j.dofadr[0] for j in fingers])
            self.hand_effort[side] = np.array(
                [m.jnt_actfrcrange[j.id][1] if m.jnt_actfrclimited[j.id] else HAND_EFFORT for j in fingers]
            )
            self.hand_tau[side] = np.zeros(NUM_HAND_JOINT)
        self.pelvis = m.body("pelvis").id
        self.mass = float(m.body_subtreemass[self.pelvis])
        self.sensor = {n: slice(m.sensor(n).adr[0], m.sensor(n).adr[0] + m.sensor(n).dim[0])
                       for n in ("imu_quat", "imu_gyro", "imu_acc")}  # fmt: skip
        self.gantry = True
        self.clearance = clearance
        self.tau = np.zeros(NUM_JOINT)
        self.reset()

    @property
    def dt(self):
        return self.model.opt.timestep

    def reset(self):
        """Joints at zero, hanging with the feet `clearance` above the floor."""
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[2] = 1.0
        mujoco.mj_kinematics(self.model, self.data)
        lowest = min(
            self.data.geom_xpos[g][2] - self.model.geom_rbound[g]
            for g in range(self.model.ngeom)
            if self.model.geom_bodyid[g] != 0 and self.model.geom_contype[g]
        )
        self.anchor = np.array([0.0, 0.0, 1.0 - lowest + self.clearance])
        self.data.qpos[:3] = self.anchor
        self.tau[:] = 0.0
        for tau in self.hand_tau.values():
            tau[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def step(self, cmd, hand_cmds=None):
        """One physics step under `cmd` and `hand_cmds` ({side: HandCommand}); torques clipped."""
        d = self.data
        q, dq = d.qpos[self.qadr], d.qvel[self.vadr]
        tau = cmd.tau + cmd.kp * (cmd.q - q) + cmd.kd * (cmd.dq - dq)
        tau = np.where(cmd.mode != 0, tau, 0.0)
        self.tau = np.clip(tau, -self.effort, self.effort)
        d.qfrc_applied[:] = 0.0
        d.qfrc_applied[self.vadr] = self.tau
        for side, hand in (hand_cmds or {}).items() if self.hands else ():
            qadr, vadr, limit = self.hand_qadr[side], self.hand_vadr[side], self.hand_effort[side]
            torque = hand.tau + hand.kp * (hand.q - d.qpos[qadr]) + hand.kd * (hand.dq - d.qvel[vadr])
            self.hand_tau[side] = np.clip(torque, -limit, limit)
            d.qfrc_applied[vadr] = self.hand_tau[side]
        d.xfrc_applied[self.pelvis] = self._gantry_wrench() if self.gantry else 0.0
        mujoco.mj_step(self.model, d)

    def _gantry_wrench(self):
        d = self.data
        kp, kr = self.GANTRY_KP
        dp, dr = self.GANTRY_KD
        force = kp * (self.anchor - d.qpos[:3]) - dp * d.qvel[:3]
        force[2] += self.mass * -self.model.opt.gravity[2]
        err = np.zeros(3)
        mujoco.mju_subQuat(err, np.array([1.0, 0, 0, 0]), d.qpos[3:7])
        torque = kr * err - dr * d.qvel[3:6]  # free-joint angular velocity is in the body frame
        return np.concatenate([force, d.xmat[self.pelvis].reshape(3, 3) @ torque])

    def hand_state(self, side):
        """HandState fields of one Dex3: motor q/dq/tau_est (7)."""
        d = self.data
        return {
            "q": d.qpos[self.hand_qadr[side]].copy(),
            "dq": d.qvel[self.hand_vadr[side]].copy(),
            "tau_est": self.hand_tau[side].copy(),
        }

    def state(self):
        """LowState fields: motor q/dq/ddq/tau_est (29) and the pelvis IMU."""
        d = self.data
        quat = d.sensordata[self.sensor["imu_quat"]].copy()
        return {
            "q": d.qpos[self.qadr].copy(),
            "dq": d.qvel[self.vadr].copy(),
            "ddq": d.qacc[self.vadr].copy(),
            "tau_est": self.tau.copy(),
            "quaternion": quat,
            "rpy": rpy_from_quat(quat),
            "gyroscope": d.sensordata[self.sensor["imu_gyro"]].copy(),
            "accelerometer": d.sensordata[self.sensor["imu_acc"]].copy(),
            "time": d.time,
        }
