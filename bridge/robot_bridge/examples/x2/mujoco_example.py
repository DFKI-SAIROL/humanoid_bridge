import argparse
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
import rclpy
import yaml
from robot_bridge.robot_client import RobotClient


class RobotController:
    def __init__(self, node, model_path, config_path, use_imu=False):
        self.node = node
        with open(config_path) as file:
            config = yaml.safe_load(file)["robot_bridge"]["ros__parameters"]
        self.num_dof = config["num_joint"]
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        self.use_imu = use_imu
        if self.use_imu:
            base_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "floating_base_joint")
            if base_id < 0 or self.model.jnt_type[base_id] != mujoco.mjtJoint.mjJNT_FREE:
                raise ValueError("IMU display requires floating_base_joint.")
            self.base_quat_index = self.model.jnt_qposadr[base_id] + 3
        self.qpos_index = []
        self.state_index = []
        for name in config["joint_names"]:
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id < 0 or self.model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
                raise ValueError(f"Missing hinge joint in model: {name}")
            self.qpos_index.append(self.model.jnt_qposadr[joint_id])
            self.state_index.append(config[name]["idx"])

        self.robot = RobotClient(node=self.node, robot_type="X2", num_dof=self.num_dof, control_frequency=50.0, interpolation_order=0.0)
        # The read-only viewer must not advertise a command publisher.
        self.node.destroy_publisher(self.robot.low_cmd_publisher)
        self.robot.low_cmd_publisher = None
        self.viewer = None
        self.timer = self.node.create_timer(0.02, self.step)  # 50 Hz display
        mujoco.mj_forward(self.model, self.data)
        print("Read-only X2 viewer. No commands or mode requests are sent.")
        if self.use_imu:
            print("Pelvis IMU orientation enabled; base position is fixed.")
        else:
            print("The base is fixed; IMU orientation is not applied.")
        print("Values may be initial zeros or stale data; RobotClient does not check freshness.")

    def step(self):
        if self.viewer is None:
            return
        with self.viewer.lock():
            # Keep the floating base and unmapped joints at the model defaults
            self.data.qpos[:] = self.model.qpos0
            self.data.qpos[self.qpos_index] = self.robot.q_pos[self.state_index]
            self.data.qvel[:] = 0.0
            mujoco.mj_forward(self.model, self.data)
            if self.use_imu:
                quat = np.asarray(self.robot.quat, dtype=np.float64)
                norm = np.linalg.norm(quat)
                if np.all(np.isfinite(quat)) and norm > 1e-12:
                    # The HAL torso topic provides pelvis orientation in w, x, y, z order.
                    self.data.qpos[self.base_quat_index:self.base_quat_index + 4] = quat / norm
                    mujoco.mj_forward(self.model, self.data)
        self.viewer.sync()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Display X2 joint feedback in MuJoCo without controlling the robot.")
    parser.add_argument("--model", type=Path, default=Path.home() / "robotics/agibot_x2_urdf/X2_URDF-v1.3.0/scene.xml")
    parser.add_argument("--config", type=Path, default=Path(__file__).resolve().parents[2] / "params/X2_config.yaml")
    parser.add_argument("--use-imu", action="store_true", help="Apply pelvis IMU orientation while keeping base position fixed.")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("x2_mujoco_viewer")
    try:
        controller = RobotController(node, args.model, args.config, args.use_imu)
        with mujoco.viewer.launch_passive(controller.model, controller.data) as viewer:
            controller.viewer = viewer
            while rclpy.ok() and viewer.is_running():
                rclpy.spin_once(node, timeout_sec=0.02)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
