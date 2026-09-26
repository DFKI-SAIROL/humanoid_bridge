import argparse
from pathlib import Path

import mujoco
import mujoco.viewer
import rclpy
import yaml
from robot_bridge_py.robot_client import RobotClient


class RobotController:
    def __init__(self, node, model_path, config_path):
        self.node = node
        with open(config_path) as file:
            config = yaml.safe_load(file)["robot_bridge"]["ros__parameters"]
        self.num_dof = config["num_joint"]
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        self.qpos_index = []
        self.state_index = []
        for name in config["joint_names"]:
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id < 0 or self.model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
                raise ValueError(f"Missing hinge joint in model: {name}")
            self.qpos_index.append(self.model.jnt_qposadr[joint_id])
            self.state_index.append(config[name]["idx"])

        self.robot = RobotClient(node=self.node, robot_type="X2", num_dof=self.num_dof, control_frequency=50.0, interpolation_order=0.0)
        self.viewer = None
        self.timer = self.node.create_timer(0.02, self.step)  # 50 Hz display
        mujoco.mj_forward(self.model, self.data)
        print("Read-only X2 viewer. No commands or mode requests are sent.")
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
        self.viewer.sync()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Display X2 joint feedback in MuJoCo without controlling the robot.")
    parser.add_argument("--model", type=Path, default=Path.home() / "robotics/agibot_x2_urdf/X2_URDF-v1.3.0/scene.xml")
    parser.add_argument("--config", type=Path, default=Path(__file__).resolve().parents[2] / "params/X2_config.yaml")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("x2_mujoco_viewer")
    try:
        controller = RobotController(node, args.model, args.config)
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
