"""X2 walking policy example. Adapt model loading and policy_step for another policy."""

import time
from pathlib import Path

import numpy as np
import onnxruntime as ort

from config import Config
from utils.robot_client import PolicyRobotClient
from utils.control import ControlSession, load_bridge_config, run_controller
from utils.control_log import ControlLog
from utils.policy_helper import create_history, build_observation, get_gravity_orientation


class RobotController:
    def __init__(self, node, cfg: Config):
        self.config = load_bridge_config()
        config = self.config
        self.node = node
        self.exit_reason = "normal exit"
        self.recorder = ControlLog(cfg, config)
        self.names = config["joint_names"]
        self.policy_config = cfg
        expected = [n for area in cfg.joint_areas.values() for n in area["joints"] if n != "head_pitch_joint"]
        if config["num_joint"] != 30 or self.names != expected or [config[n]["idx"] for n in self.names] != list(range(30)):
            raise ValueError("This client requires the X2 30-joint bridge order, including head yaw.")
        self.obs_names = cfg.obs_joint_names
        self.action_names = cfg.action_joint_names
        if len(self.obs_names) != 31 or set(self.obs_names) != set(self.names + ["head_pitch_joint"]):
            raise ValueError("Unsupported policy observation joint order.")
        if len(self.action_names) != 29 or set(self.action_names) != set(self.names) - {"head_yaw_joint"}:
            raise ValueError("Unsupported policy action joint order.")
        self.obs_indices = [self.names.index(n) if n in self.names else 30 for n in self.obs_names]
        self.action_indices = [self.names.index(n) for n in self.action_names]
        self.default_q = np.array([cfg.default_joint_pos.get(n, 0.0) for n in self.names], dtype=np.float64)
        self.obs_default = np.array([cfg.default_joint_pos.get(n, 0.0) for n in self.obs_names])
        self.q_min = np.array([config[n]["q_min"] for n in self.names])
        self.q_max = np.array([config[n]["q_max"] for n in self.names])
        gains = cfg.rl_gains
        self.kp = np.array([gains[n]["kp"] for n in self.names])
        self.kd = np.array([gains[n]["kd"] for n in self.names])
        # Load the policy model and check its input/output contract.
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        self.session = ort.InferenceSession(str(cfg.policy_path), options,
                                            providers=["CPUExecutionProvider"])
        inputs, outputs = self.session.get_inputs(), self.session.get_outputs()
        if (len(inputs) != 1 or inputs[0].shape != [1, 500] or inputs[0].type != "tensor(float)" or
                len(outputs) != 1 or outputs[0].shape != [1, 29] or outputs[0].type != "tensor(float)" or
                sum(self.policy_config.obs_term_dims.values()) * self.policy_config.history_length != 500):
            raise ValueError("Expected a float32 policy with input [1, 500] and output [1, 29].")
        self.input_name, self.output_name = inputs[0].name, outputs[0].name
        self.robot = PolicyRobotClient(node, config, cfg.joint_areas)
        self.control = ControlSession(self)
        self.reset_history()
        self.timer = self.node.create_timer(cfg.control_dt, self.step)
        self.control.announce()

    def step(self):
        now = time.monotonic()
        if not self.check_state(now):
            return
        if self.control.send_preparation_command(now):
            return
        frame = self.recorder.capture(self, "policy", self.target_q)
        started = time.perf_counter()
        self.policy_step()
        elapsed_ms = (time.perf_counter() - started) * 1000
        if self.control.agent_started:
            # Policy targets are PD references; Bridge limits estimated torque.
            if not np.all(np.isfinite(self.target_q)):
                raise RuntimeError("Non-finite policy joint target.")
            self.robot.send_cmd(self.target_q, target_kp=self.kp, target_kd=self.kd)
            frame["sent"] = True
        self.recorder.report(self, now, elapsed_ms)

    def policy_step(self):
        # Build the model input in observation-joint order.
        q = np.append(self.robot.q_pos, self.robot.head_pitch[0])[self.obs_indices]
        dq = np.append(self.robot.q_vel, self.robot.head_pitch[1])[self.obs_indices]
        gravity = get_gravity_orientation(self.robot.quat)
        terms = {"base_ang_vel": self.robot.angular_velocity * self.policy_config.ang_vel_scale,
                 "projected_gravity": gravity, "velocity_commands": self.control.velocity_cmd.copy(),
                 "joint_pos_rel": q - self.obs_default, "joint_vel_rel": dq * self.policy_config.dof_vel_scale,
                 "last_action": self.last_action.copy()}
        self.obs = build_observation(terms, self.histories, self.policy_config.obs_term_dims,
                                     self.policy_config.obs_clip)
        self.recorder.frames[-1]["obs"] = self.obs[0].copy()
        # Run the policy network.
        output = self.session.run([self.output_name], {self.input_name: self.obs})[0]
        self.recorder.frames[-1]["raw_action"] = output[0].copy()
        if not np.all(np.isfinite(output)):
            raise ValueError("Policy produced non-finite actions.")
        action = np.clip(output[0], -self.policy_config.action_clip, self.policy_config.action_clip)
        if self.counter < self.policy_config.history_length:
            action[:] = 0.0
        self.last_action[:] = action
        # The deployment example limits target position increments, in radians.
        for i, index in enumerate(self.action_indices):
            desired = float(action[i]) * self.policy_config.action_scale + self.default_q[index]
            self.target_q[index] += np.clip(desired - self.target_q[index],
                                            -self.policy_config.action_delta_limit, self.policy_config.action_delta_limit)
        self.counter += 1
        self.recorder.frames[-1].update(action=self.last_action.copy(), target_q=self.target_q.copy())

    def check_state(self, now):
        return self.control.update(now)

    def reset_history(self):
        self.histories = create_history(self.policy_config.obs_term_dims, self.policy_config.history_length)
        self.last_action = np.zeros(29, dtype=np.float32)
        self.target_q = self.default_q.copy()
        self.counter = 0


if __name__ == "__main__":
    cfg_file = Path(__file__).resolve().parent / "configs/x2.yaml"
    config = Config(cfg_file)
    run_controller(RobotController, config)
