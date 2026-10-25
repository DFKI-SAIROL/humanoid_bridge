"""MuJoCo robot under the motor PD law, a gantry on its base. No ROS; the robot is a RobotSpec."""

import dataclasses
import pathlib
from collections.abc import Callable

import mujoco
import numpy as np

IMU_SENSORS = ("imu_quat", "imu_gyro", "imu_acc")


@dataclasses.dataclass(frozen=True)
class RobotSpec:
    """What the physics needs of one robot; robots/<name>.py fills it."""

    name: str
    joint_names: tuple  # the bridge's motor order, as the MJCF names the joints
    effort: Callable  # joint name -> effort limit, Nm
    base: str  # the body the gantry holds
    imu_site: str
    camera: str  # the MJCF camera the camera server renders
    default_mjcf: pathlib.Path
    prepare: Callable  # (MjSpec) -> None: armature, friction, camera mount, before compile
    hand_joints: dict = dataclasses.field(default_factory=dict)  # side -> names; empty: no hands
    hand_effort: float = 0.0  # Nm, for a finger the MJCF leaves unlimited
    start_q: tuple = ()  # the joints at a reset, bridge order; (): all 0
    gantry: bool = True  # a gantry holds it at start and after a reset (False: the Go2 lies)


def build_spec(robot, mjcf):
    """The robot's MJCF plus floor, IMU sensors and what robot.prepare adds."""
    mjcf = pathlib.Path(mjcf)
    if not mjcf.is_file():
        raise FileNotFoundError(
            f"{mjcf}: no {robot.name} model. Run robot_web_commander/scripts/fetch_assets.sh, "
            "or pass --mjcf."
        )
    spec = mujoco.MjSpec.from_file(str(mjcf))
    if not (mjcf.parent / spec.meshdir).is_dir() and (mjcf.parent / "meshes").is_dir():
        spec.meshdir = "meshes"  # fetch_assets.sh renames mjlab's assets/ to meshes/

    # A scene MJCF brings its own plane (Booster's T1: `ground`): a second one z-fights.
    if not any(geom.type == mujoco.mjtGeom.mjGEOM_PLANE for geom in spec.geoms):
        spec.add_texture(name="floor", type=mujoco.mjtTexture.mjTEXTURE_2D,
                         builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER, width=300, height=300,
                         rgb1=[0.2, 0.3, 0.4], rgb2=[0.1, 0.2, 0.3])  # fmt: skip
        floor = spec.add_material(name="floor", texrepeat=[5, 5], texuniform=True)
        floor.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "floor"
        spec.worldbody.add_geom(
            name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[0, 0, 0.05], material="floor"
        )
        spec.worldbody.add_light(pos=[0, 0, 4], dir=[0, 0, -1], diffuse=[0.7, 0.7, 0.7])
    robot.prepare(spec)
    sensor = mujoco.mjtSensor
    kinds = (sensor.mjSENS_FRAMEQUAT, sensor.mjSENS_GYRO, sensor.mjSENS_ACCELEROMETER)
    for name, kind in zip(IMU_SENSORS, kinds, strict=True):
        site = mujoco.mjtObj.mjOBJ_SITE
        spec.add_sensor(name=name, type=kind, objtype=site, objname=robot.imu_site)
    return spec


def rpy_from_quat(q):
    """Roll, pitch, yaw (ZYX) from w, x, y, z."""
    w, x, y, z = q
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


class Command:
    """One frame of motor targets; mode 0 is a disabled motor."""

    def __init__(self, n):
        self.mode = np.zeros(n, dtype=np.int32)
        self.q = np.zeros(n)
        self.dq = np.zeros(n)
        self.kp = np.zeros(n)
        self.kd = np.zeros(n)
        self.tau = np.zeros(n)


class HandCommand:
    """One hand's finger targets; zero gains and torque until the first one: slack."""

    def __init__(self, n):
        self.q = np.zeros(n)
        self.dq = np.zeros(n)
        self.kp = np.zeros(n)
        self.kd = np.zeros(n)
        self.tau = np.zeros(n)


class RobotSim:
    """Steps the robot with the motor PD law; a gantry holds the base while it is on.

    `hands`: the MJCF has every joint of robot.hand_joints; `step` then takes a HandCommand a side.
    """

    GANTRY_KP = (4000.0, 400.0)  # N/m, Nm/rad
    GANTRY_KD = (400.0, 40.0)

    def __init__(self, robot, mjcf=None, clearance=0.05):
        self.robot = robot
        self.model = build_spec(robot, mjcf or robot.default_mjcf).compile()
        self.data = mujoco.MjData(self.model)
        m = self.model
        joints = [m.joint(n) for n in robot.joint_names]
        self.num_joint = len(joints)
        self.qadr = np.array([j.qposadr[0] for j in joints])
        self.vadr = np.array([j.dofadr[0] for j in joints])
        self.effort = np.array([robot.effort(n) for n in robot.joint_names])
        self.hands = bool(robot.hand_joints) and all(
            mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) >= 0
            for names in robot.hand_joints.values() for n in names
        )  # fmt: skip
        self.hand_qadr, self.hand_vadr, self.hand_effort, self.hand_tau = {}, {}, {}, {}
        for side, names in robot.hand_joints.items() if self.hands else ():
            fingers = [m.joint(n) for n in names]
            self.hand_qadr[side] = np.array([j.qposadr[0] for j in fingers])
            self.hand_vadr[side] = np.array([j.dofadr[0] for j in fingers])
            self.hand_effort[side] = np.array(
                [m.jnt_actfrcrange[j.id][1] if m.jnt_actfrclimited[j.id] else robot.hand_effort
                 for j in fingers]
            )  # fmt: skip
            self.hand_tau[side] = np.zeros(len(fingers))
        self.base = m.body(robot.base).id
        self.mass = float(m.body_subtreemass[self.base])
        self.sensor = {n: slice(m.sensor(n).adr[0], m.sensor(n).adr[0] + m.sensor(n).dim[0])
                       for n in IMU_SENSORS}  # fmt: skip
        self.gantry = True
        self.clearance = clearance
        self.tau = np.zeros(self.num_joint)
        self.reset()

    @property
    def dt(self):
        return self.model.opt.timestep

    def command(self):
        """A disabled Command of this robot's size."""
        return Command(self.num_joint)

    def hand_command(self, side):
        """A slack HandCommand for one side."""
        return HandCommand(len(self.robot.hand_joints[side]))

    def reset(self):
        """Joints at start_q (else 0), the lowest point `clearance` above the floor."""
        mujoco.mj_resetData(self.model, self.data)
        if self.robot.start_q:
            self.data.qpos[self.qadr] = self.robot.start_q
        self.data.qpos[2] = 1.0
        mujoco.mj_kinematics(self.model, self.data)
        m = self.model
        body = [g for g in range(m.ngeom) if m.geom_bodyid[g] != 0]
        # Colliding geoms; a model that collides by <pair> only (ncbf's T1): every robot geom.
        colliding = [g for g in body if m.geom_contype[g] or m.geom_conaffinity[g]] or body
        lowest = min(self.data.geom_xpos[g][2] - m.geom_rbound[g] for g in colliding)
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
            q, dq = d.qpos[qadr], d.qvel[vadr]
            torque = hand.tau + hand.kp * (hand.q - q) + hand.kd * (hand.dq - dq)
            self.hand_tau[side] = np.clip(torque, -limit, limit)
            d.qfrc_applied[vadr] = self.hand_tau[side]
        d.xfrc_applied[self.base] = self._gantry_wrench() if self.gantry else 0.0
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
        return np.concatenate([force, d.xmat[self.base].reshape(3, 3) @ torque])

    def hand_state(self, side):
        """One hand's finger q/dq/tau_est."""
        d = self.data
        return {
            "q": d.qpos[self.hand_qadr[side]].copy(),
            "dq": d.qvel[self.hand_vadr[side]].copy(),
            "tau_est": self.hand_tau[side].copy(),
        }

    def state(self):
        """Motor q/dq/ddq/tau_est in the bridge order and the IMU."""
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
