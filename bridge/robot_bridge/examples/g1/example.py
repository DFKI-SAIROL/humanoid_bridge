import rclpy
import time
import pathlib
import numpy as np
import yaml
from robot_bridge.robot_client import RobotClient, KeyMap # ,WirelessKey_H1_G1
from enum import Enum
import torch
from utils.rotation_helper import get_gravity_orientation, transform_imu_data
from config import Config

class RobotController:   
    def __init__(self, node, cfg: Config):
        self.config = cfg
        self.node = node  
        self.robot = RobotClient(node=self.node, robot_type="G1", num_dof=29, control_frequency=50.0, interpolation_order=0.0)
        self.timer = self.node.create_timer(0.02, self.step)  # 50 Hz control frequency
        self.action = np.zeros(cfg.num_actions, dtype=np.float32)
        # Initialize components
        self.policy = torch.jit.load(cfg.policy_path)
        self.counter = 0

        self.init_pos = np.concatenate([self.config.default_angles, self.config.arm_waist_target]).astype(np.float32)
        self.policy_kp = np.concatenate([self.config.kps, self.config.arm_waist_kps]).astype(np.float32)
        self.policy_kd = np.concatenate([self.config.kds, self.config.arm_waist_kds]).astype(np.float32)

        self.obs = np.zeros(cfg.num_obs, dtype=np.float32)
        self.agent_started = False
        self.vx_cmd = 0.0
        self.vy_cmd = 0.0
        self.vyaw_cmd = 0.0
        print("Please press\n\t \"L2 + START\" to start control, \n\t \"R2 + A\" to start inferrence, \n\t \"L2 + left + up\" to stop control,\n\t ready and zero position: /ready_position_control, /zero_position_control.")

    def step(self):
        self.check_state()
        if self.robot.control_started and self.agent_started:
            self.policy_step()  
            
    def policy_step(self):
        self.counter += 1
        
        q = self.robot.q_pos[self.config.leg_joint2motor_idx]
        dq = self.robot.q_vel[self.config.leg_joint2motor_idx]
        
        # imu_state quaternion: w, x, y, z
        ang_vel = self.robot.angular_velocity
        quat = self.robot.quat

        if self.config.imu_type == "torso":
            # h1 and h1_2 imu is on the torso
            # imu data needs to be transformed to the pelvis frame
            waist_yaw = self.robot.q_pos[self.config.arm_waist_joint2motor_idx[0]]
            waist_yaw_omega = self.robot.q_vel[self.config.arm_waist_joint2motor_idx[0]]
            quat, ang_vel = transform_imu_data(waist_yaw=waist_yaw, waist_yaw_omega=waist_yaw_omega, imu_quat=quat, imu_omega=ang_vel)

        # create observation
        gravity_orientation = get_gravity_orientation(quat)
        qj_obs = q
        dqj_obs = dq
        qj_obs = (qj_obs - self.config.default_angles) * self.config.dof_pos_scale
        dqj_obs = dqj_obs * self.config.dof_vel_scale
        ang_vel = ang_vel * self.config.ang_vel_scale
        period = 0.8
        count = self.counter * self.config.control_dt
        phase = count % period / period
        sin_phase = np.sin(2 * np.pi * phase)
        cos_phase = np.cos(2 * np.pi * phase)
        
        
        num_actions = self.config.num_actions
        self.obs[:3] = ang_vel
        self.obs[3:6] = gravity_orientation
        self.obs[6:9] = np.array([self.vx_cmd, self.vy_cmd, self.vyaw_cmd]) * self.config.cmd_scale * self.config.max_cmd
        self.obs[9 : 9 + num_actions] = qj_obs
        self.obs[9 + num_actions : 9 + num_actions * 2] = dqj_obs
        self.obs[9 + num_actions * 2 : 9 + num_actions * 3] = self.action
        self.obs[9 + num_actions * 3] = sin_phase
        self.obs[9 + num_actions * 3 + 1] = cos_phase

        # Get the action from the policy network
        obs_tensor = torch.from_numpy(self.obs).unsqueeze(0)
        self.action = self.policy(obs_tensor).detach().numpy().squeeze()
        
        # transform action to target_dof_pos
        target_dof_pos = self.config.default_angles + self.action * self.config.action_scale


        q_target_pos = np.concatenate([target_dof_pos, self.config.arm_waist_target])
    
        # send the command
        self.robot.send_cmd(q_target_pos, target_kp=self.policy_kp, target_kd=self.policy_kd)

    def check_state(self):
        self.robot.update_robot_state()

        if self.robot.joy_key is not None:

            # if (self.robot.joy_key.keys & (WirelessKey_H1_G1.KEY_R2 | WirelessKey_H1_G1.KEY_A)) == (WirelessKey_H1_G1.KEY_R2 | WirelessKey_H1_G1.KEY_A):  # start: R2 + A
            if self.robot.remote_controller.is_exact_combo(self.robot.joy_key, [KeyMap.R2, KeyMap.A]):  # start: R2 + A
                if self.robot.control_started:
                    self.agent_started = True
                    self.vx_cmd = 0.0
                    self.vy_cmd = 0.0
                    self.vyaw_cmd = 0.0
                    self.node.get_logger().info("Agent started.")
                else:
                    self.node.get_logger().warn("Please start the control first by pressing L2 + START.")
            
            self.robot.joy_key = None  # Reset joy_key

        if self.agent_started:
            # self.vx_cmd = self.robot.joy_key.ly
            # self.vy_cmd = self.robot.joy_key.lx * -1
            # self.vyaw_cmd = self.robot.joy_key.rx * -1
            
            self.vx_cmd = self.robot.remote_controller.ly
            self.vy_cmd = self.robot.remote_controller.lx * -1
            self.vyaw_cmd = self.robot.remote_controller.rx * -1
            
            self.vx_cmd = self.vx_cmd * 0.5
            self.vy_cmd = self.vy_cmd * 0.5
            self.vyaw_cmd = self.vyaw_cmd * 0.4
            print(f"Velocity commands - vx: {self.vx_cmd}, vy: {self.vy_cmd}, vyaw: {self.vyaw_cmd}")
        else:
            self.vx_cmd = 0.0
            self.vy_cmd = 0.0
            self.vyaw_cmd = 0.0

        if not self.robot.control_started:
            self.agent_started = False


if __name__ == "__main__":
    rclpy.init()
    # Beside this file, not relative to the working directory.
    cfg_file = str(pathlib.Path(__file__).parent / "configs" / "g1.yaml")
    config = Config(cfg_file)
    node = rclpy.create_node('robot_client_node')
    controller = RobotController(node, config)
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()