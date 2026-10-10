"""X2 policy client: preview by default, joystick control with --enable-control."""

import argparse
from collections import deque
from datetime import datetime
import hashlib
import json
import time
from pathlib import Path

import numpy as np
import onnxruntime as ort
import rclpy
import yaml
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import parameter_value_to_python
from rcl_interfaces.srv import GetParameters
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Joy
from robot_bridge_py.robot_client import RemoteController, KeyMap

from config import Config
from utils.robot_client import PolicyRobotClient
from utils.policy_helper import create_history, build_observation, get_gravity_orientation


class RobotController:
    def __init__(self, node, cfg: Config, args, config):
        self.node = node
        self.config = config
        self.enable_control = args.enable_control
        self.record_path = getattr(args, "record", None)
        self.record_started = None
        self.record_frames = []
        self.control_frames = deque(maxlen=251)
        self.exit_reason = "normal exit"
        self.control_metadata = {
            "model_sha256": hashlib.sha256(cfg.policy_path.read_bytes()).hexdigest(),
            "bridge_config": config,
            "policy_config": yaml.safe_load((Path(__file__).resolve().parent / "configs/x2.yaml"
                                              if not hasattr(args, "policy_config") else Path(args.policy_config)).read_text()),
            "joint_names": config["joint_names"], "obs_joint_names": cfg.obs_joint_names,
            "action_joint_names": cfg.action_joint_names,
            "units": "rad, rad/s; quaternion wxyz; monotonic seconds",
            "note": "sent means client send_cmd returned, not hardware acknowledgement; NaN means not computed."}
        if self.record_path is not None:
            if self.enable_control:
                raise ValueError("Recording is only available in read-only mode.")
            self.record_path = Path(self.record_path).expanduser().resolve()
            if self.record_path.exists():
                raise ValueError(f"Recording already exists: {self.record_path}")
            self.record_metadata = {
                "model_sha256": hashlib.sha256(cfg.policy_path.read_bytes()).hexdigest(),
                "policy_config": yaml.safe_load(Path(args.policy_config).read_text()),
                "bridge_config": config,
                "joint_names": config["joint_names"], "obs_joint_names": cfg.obs_joint_names,
                "action_joint_names": cfg.action_joint_names,
                "units": "angles: rad; joint/angular velocities: rad/s; quaternion: wxyz; time: monotonic seconds",
                "note": "Read-only snapshots after client validation; no commands sent. No raw ROS headers recorded."}
            print(f"将记录约 10 秒有效状态与模型计算，保存到：{self.record_path}")
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
        self.robot = PolicyRobotClient(node, config, cfg.joint_areas, self.enable_control)
        self.init_pos = np.asarray(config["ready_q"], dtype=np.float64)
        self.prep_kp = np.array([config[n]["kp"] for n in self.names])
        self.prep_kd = np.array([config[n]["kd"] for n in self.names])
        self.robot._default_duration = float(config["duration"])
        self.robot.set_default_cmd(self.init_pos, self.prep_kp, self.prep_kd)
        if self.enable_control:
            self.bridge_parameters = node.create_client(GetParameters, "/robot_bridge/get_parameters")
            self.parameter_names = ["duration", "num_joint", "torque_control", "joint_names", "ready_q"]
            self.parameter_names += [f"{group}_{gain}_{bound}" for group in ("upper_limbs", "lower_limbs")
                                     for gain in ("kp", "kd") for bound in ("min", "max")]
        self.agent_started = False
        self.control_requested = False
        self.stop_requested = False
        self.pending = None
        self.ready_at = None
        self.transition_start = None
        self.joy_key = None
        self.remote = RemoteController()
        self.joy_time = 0.0
        self.velocity_cmd = np.zeros(3, dtype=np.float32)
        if args.joystick or self.enable_control:
            self.joy_subscriber = node.create_subscription(Joy, "/joy", self.joy_callback, qos_profile_sensor_data)
        self.reset_history()
        self.was_ready = False
        self.last_print = 0.0
        self.timer = node.create_timer(cfg.control_dt, self.step)
        print(f"X2 policy：{cfg.policy_path}")
        if self.enable_control:
            print("控制模式：先启动 X2 Bridge 和 game_controller_node。")
            print("L2 + START：准备姿态；R2 + ×：过渡到模型姿态后自动开启 policy。")
            print("Policy 目标超过配置角度范围仅提示；Bridge 保留力矩约束。")
            print("摇杆控制速度；L2 + UP + LEFT：停止发送并请求 Bridge 停止。")
            print("新版 X2 Bridge 收到停止请求后会锁定阻尼；不会退出调试模式。恢复控制需重启 Bridge。")
        else:
            print("只读模式：不发命令、不调用控制服务，不需要启动 Bridge。")
        print("50 Hz policy；头 pitch 必须有有效反馈。")
        self.report_gain_limits()

    def report_gain_limits(self):
        clipped = []
        for i in range(30):
            prefix = "lower_limbs" if self.config[self.names[i]]["if_strong_joint"] else "upper_limbs"
            if any(not self.config[f"{prefix}_{key}_min"] <= value <= self.config[f"{prefix}_{key}_max"]
                   for key, value in (("kp", self.kp[i]), ("kd", self.kd[i]))):
                clipped.append(self.names[i])
        if clipped:
            message = "Policy gains exceed bridge configuration: " + ", ".join(clipped)
            if self.enable_control:
                raise ValueError(message)
            print(message)
        if self.enable_control:
            self.check_target(self.init_pos)
            self.check_target(self.default_q)

    def joy_callback(self, msg):
        if self.remote.set_joy(msg):
            # H1 stick signs with SDK velocity command ranges.
            self.velocity_cmd[:] = [np.clip(self.remote.ly * 0.5, -0.3, 0.5),
                                    -self.remote.lx * 0.3, -self.remote.rx * 0.2]
            self.joy_time = time.monotonic()
            if self.remote.new_event:
                self.joy_key = self.remote.button.copy()
        else:
            self.velocity_cmd[:] = 0.0
            self.joy_time = 0.0
            self.joy_key = None

    def check_target(self, target):
        if not np.all(np.isfinite(target)):
            raise RuntimeError("Non-finite joint target.")
        outside = [self.names[i] for i in range(30) if not self.q_min[i] <= target[i] <= self.q_max[i]]
        if outside:
            details = []
            for name in outside:
                i = self.names.index(name)
                actual, desired, lower, upper = np.rad2deg(
                    [self.robot.q_pos[i], target[i], self.q_min[i], self.q_max[i]])
                details.append(f"{name}: actual={actual:+.3f} deg, target={desired:+.3f} deg, "
                               f"limits=[{lower:+.3f}, {upper:+.3f}] deg, "
                               f"excess={max(lower - desired, desired - upper):.3f} deg")
            raise RuntimeError("Joint target outside configured limits: " + "; ".join(details))

    def request_stop(self):
        self.agent_started = False
        self.transition_start = None
        self.ready_at = None
        self.robot.control_started = False
        self.robot.control_start_time = None
        self.velocity_cmd[:] = 0.0
        self.joy_key = None
        self.stop_requested = True
        # Let a pending start finish before sending stop, so start cannot overtake it.
        if self.control_requested and self.pending is None:
            self.pending = self.robot.stop_control()
            self.request_kind = "stop"
            self.request_time = time.monotonic()

    def check_state(self):
        now = time.monotonic()
        if self.pending is not None:
            if not self.pending.done():
                if now - self.request_time > 5.0:
                    raise RuntimeError(f"Bridge {self.request_kind} response timed out; robot-side takeover required.")
                return
            response = self.pending.result()
            self.pending = None
            if self.request_kind == "parameters":
                if self.stop_requested:
                    return
                actual = [parameter_value_to_python(value) for value in response.values]
                if actual != [self.config[name] for name in self.parameter_names]:
                    raise RuntimeError("Bridge parameters differ; restart Bridge with the same X2_config.yaml.")
                if self.robot.unavailable_groups() or self.node.count_publishers("/robot_cmd") != 1:
                    raise RuntimeError("Feedback or command ownership changed before start.")
                self.check_target(self.robot.q_pos)
                self.control_requested = True
                self.pending = self.robot.init_control(self.init_pos)
                self.request_kind = "start"
                self.request_time = now
                return
            if not response.success:
                raise RuntimeError(f"Bridge {self.request_kind} rejected: {response.message}")
            if self.request_kind == "stop":
                self.control_requested = False
                self.stop_requested = False
                print("Bridge 停止请求已确认；新版 X2 Bridge 将持续发阻尼。此回复不代表真机已停稳。", flush=True)
                return
            elif self.stop_requested:
                self.request_stop()
                return
            else:
                self.ready_at = now + self.robot._default_duration
        if self.ready_at is not None and now >= self.ready_at:
            self.ready_at = None
            self.robot.control_started = True
            print("准备姿态阶段完成；按 R2 + × 过渡到模型姿态，随后自动开启 policy。", flush=True)
        buttons, self.joy_key = self.joy_key, None
        if buttons is None or now - self.joy_time > 0.5:
            return
        combo = self.remote.is_exact_combo
        if combo(buttons, [KeyMap.L2, KeyMap.up, KeyMap.left]):
            self.request_stop()
        elif combo(buttons, [KeyMap.L2, KeyMap.start]) and not self.control_requested:
            if self.robot.unavailable_groups():
                self.node.get_logger().warn("Waiting for valid robot feedback.")
                return
            self.check_target(self.robot.q_pos)
            if self.node.count_publishers("/robot_cmd") != 1:
                raise RuntimeError("Close other /robot_cmd clients before starting.")
            if not self.robot.stop_control_client.service_is_ready():
                raise RuntimeError("Bridge stop service is unavailable.")
            self.stop_requested = False
            if not self.bridge_parameters.service_is_ready():
                raise RuntimeError("Bridge parameter service is unavailable.")
            self.pending = self.bridge_parameters.call_async(GetParameters.Request(names=self.parameter_names))
            self.request_kind = "parameters"
            self.request_time = now
        elif combo(buttons, [KeyMap.R2, KeyMap.A]) and self.robot.control_started:
            if self.agent_started or self.transition_start is not None:
                return
            if np.any(np.abs(self.velocity_cmd) > 0.02):
                self.node.get_logger().warn("Center the sticks before starting policy.")
                return
            self.transition_q = self.robot.q_pos.copy()
            self.transition_start = now
            self.reset_history()
            self.velocity_cmd[:] = 0.0
            print("正在过渡到模型默认姿态，1.5 秒后自动开启 policy。", flush=True)

    def reset_history(self):
        self.histories = create_history(self.policy_config.obs_term_dims, self.policy_config.history_length)
        self.last_action = np.zeros(29, dtype=np.float32)
        self.target_q = self.default_q.copy()
        self.counter = 0

    def policy_step(self):
        q = np.append(self.robot.q_pos, self.robot.head_pitch[0])[self.obs_indices]
        dq = np.append(self.robot.q_vel, self.robot.head_pitch[1])[self.obs_indices]
        gravity = get_gravity_orientation(self.robot.quat)
        terms = {"base_ang_vel": self.robot.angular_velocity * self.policy_config.ang_vel_scale,
                 "projected_gravity": gravity, "velocity_commands": self.velocity_cmd.copy(),
                 "joint_pos_rel": q - self.obs_default, "joint_vel_rel": dq * self.policy_config.dof_vel_scale,
                 "last_action": self.last_action.copy()}
        self.obs = build_observation(terms, self.histories, self.policy_config.obs_term_dims,
                                     self.policy_config.obs_clip)
        if self.enable_control:
            self.control_frames[-1]["obs"] = self.obs[0].copy()
        output = self.session.run([self.output_name], {self.input_name: self.obs})[0]
        if self.enable_control:
            self.control_frames[-1]["raw_action"] = output[0].copy()
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
        if self.enable_control:
            self.control_frames[-1].update(action=self.last_action.copy(), target_q=self.target_q.copy())
        if self.record_path is not None:
            now = time.monotonic()
            if self.record_started is None:
                self.record_started = now
            frame = {"time": now, "step": self.counter, "q": q, "dq": dq,
                     "quat": self.robot.quat, "angular_velocity": self.robot.angular_velocity,
                     "gravity": gravity, "velocity_cmd": self.velocity_cmd,
                     "previous_action": terms["last_action"], "obs": self.obs[0],
                     "raw_action": output[0], "action": self.last_action, "target_q": self.target_q,
                     "state_age": [now - self.robot.received[g] for g in (*self.policy_config.joint_areas, "imu")]}
            self.record_frames.append({key: np.array(value, copy=True) for key, value in frame.items()})

    def capture_control_frame(self, phase, target):
        now = time.monotonic()
        frame = {"time": now, "phase": phase, "step": self.counter, "sent": False,
                 "q": self.robot.q_pos.copy(), "dq": self.robot.q_vel.copy(),
                 "head_pitch": self.robot.head_pitch.copy(), "quat": self.robot.quat.copy(),
                 "angular_velocity": self.robot.angular_velocity.copy(),
                 "velocity_cmd": self.velocity_cmd.copy(), "target_q": target.copy(),
                 "previous_action": self.last_action.copy(), "obs": np.full(500, np.nan),
                 "raw_action": np.full(29, np.nan), "action": np.full(29, np.nan),
                 "state_age": np.array([now - self.robot.received[g]
                                        for g in (*self.policy_config.joint_areas, "imu")])}
        self.control_frames.append(frame)
        while self.control_frames and now - self.control_frames[0]["time"] > 5.0:
            self.control_frames.popleft()
        return frame

    def save_control_recording(self):
        if not self.enable_control or not self.control_frames:
            return
        folder = Path.home() / "x2_logs"
        folder.mkdir(exist_ok=True)
        path = folder / ("policy_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".npz")
        arrays = {key: np.stack([frame[key] for frame in self.control_frames])
                  for key in self.control_frames[0]}
        metadata = dict(self.control_metadata, exit_reason=self.exit_reason,
                        sample_count=len(self.control_frames),
                        state_age_groups=[*self.policy_config.joint_areas, "imu"])
        with path.open("xb") as stream:
            np.savez_compressed(stream, metadata=json.dumps(metadata), **arrays)
        print(f"控制记录已保存 {len(self.control_frames)} 帧：{path}", flush=True)

    def save_recording(self):
        if self.record_path is None:
            return
        arrays = {key: np.stack([frame[key] for frame in self.record_frames])
                  for key in self.record_frames[0]} if self.record_frames else {}
        self.record_metadata["sample_count"] = len(self.record_frames)
        self.record_metadata["state_age_groups"] = [*self.policy_config.joint_areas, "imu"]
        # Exclusive creation protects previous captures. Metadata uses JSON, not pickle.
        with self.record_path.open("xb") as stream:
            np.savez_compressed(stream, metadata=json.dumps(self.record_metadata), **arrays)
        print(f"已保存 {len(self.record_frames)} 帧：{self.record_path}", flush=True)

    def step(self):
        now = time.monotonic()
        if now - self.joy_time > 0.5:
            self.velocity_cmd[:] = 0.0
        if self.enable_control:
            # A stop edge takes priority even while a start request is pending.
            if self.joy_key is not None and self.remote.is_exact_combo(
                    self.joy_key, [KeyMap.L2, KeyMap.up, KeyMap.left]):
                self.request_stop()
            self.check_state()
        missing = self.robot.unavailable_groups()
        if missing:
            if self.enable_control and self.control_requested:
                raise RuntimeError("Robot feedback unavailable: " + ", ".join(missing))
            if self.was_ready:
                self.reset_history()
            self.was_ready = False
            if now - self.last_print >= 1.0:
                print("等待有效反馈：" + ", ".join(missing), flush=True)
                self.last_print = now
            return
        self.was_ready = True
        if self.enable_control:
            if not self.robot.control_started or self.stop_requested:
                return
            if self.node.count_publishers("/robot_cmd") != 1:
                raise RuntimeError("Another /robot_cmd publisher appeared.")
            if self.transition_start is not None:
                phase = min((now - self.transition_start) / 1.5, 1.0)
                weight = 0.5 * (1.0 - np.cos(np.pi * phase))
                target = self.transition_q + weight * (self.default_q - self.transition_q)
                frame = self.capture_control_frame("transition", target)
                self.check_target(target)
                self.robot.send_cmd(target, target_kp=self.kp, target_kd=self.kd)
                frame["sent"] = True
                if phase >= 1.0:
                    self.transition_start = None
                    self.agent_started = True
                    self.reset_history()
                    self.velocity_cmd[:] = 0.0
                    print("Policy 已开启：历史已重置，前 5 帧暖机；摇杆控制速度。", flush=True)
                return
            if not self.agent_started:
                frame = self.capture_control_frame("preparation_hold", self.init_pos)
                self.robot.send_cmd(self.init_pos, target_kp=self.prep_kp, target_kd=self.prep_kd)
                frame["sent"] = True
                return
        if self.enable_control:
            frame = self.capture_control_frame("policy", self.target_q)
        started = time.perf_counter()
        self.policy_step()
        elapsed_ms = (time.perf_counter() - started) * 1000
        if self.enable_control and self.agent_started:
            # Like H1/SDK, policy targets are PD references; Bridge limits estimated torque.
            if not np.all(np.isfinite(self.target_q)):
                raise RuntimeError("Non-finite policy joint target.")
            self.robot.send_cmd(self.target_q, target_kp=self.kp, target_kd=self.kd)
            frame["sent"] = True
        if now - self.last_print >= 1.0:
            outside = [self.names[i] for i in self.action_indices if not self.q_min[i] <= self.target_q[i] <= self.q_max[i]]
            mode = "控制" if self.agent_started else "只读"
            print(f"{mode} #{self.counter} | 速度 {self.velocity_cmd.round(2)} | "
                  f"推理 {elapsed_ms:.1f} ms | 目标超出配置范围 {len(outside)}", flush=True)
            for name in outside:
                i = self.names.index(name)
                actual, target, lower, upper = np.rad2deg(
                    [self.robot.q_pos[i], self.target_q[i], self.q_min[i], self.q_max[i]])
                excess = max(lower - target, target - upper, 0.0)
                print(f"目标范围提示：{name}\n"
                      f"  实测 {actual:+.3f}° | 模型目标 {target:+.3f}°\n"
                      f"  配置角度范围 [{lower:+.3f}°, {upper:+.3f}°] | 目标超出 {excess:.3f}°", flush=True)
            self.last_print = now

    def stop(self):
        self.timer.cancel()
        if not self.enable_control or not self.control_requested:
            return
        self.request_stop()
        deadline = time.monotonic() + 2.0
        while self.control_requested and time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)
            self.check_state()
        if self.control_requested:
            self.node.get_logger().error("Stop not confirmed; use robot-side stop or takeover.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--policy-config", type=Path, default=Path(__file__).resolve().parent / "configs/x2.yaml",
                        help="Policy configuration file.")
    parser.add_argument("--config", type=Path, default=Path(__file__).resolve().parents[2] / "params/X2_config.yaml")
    parser.add_argument("--joystick", action="store_true", help="Read /joy stick axes in preview mode; no button actions without --enable-control.")
    parser.add_argument("--enable-control", action="store_true", help="Enable bridge commands and joystick start/stop controls.")
    parser.add_argument("--record", type=Path, help="Save a 10-second read-only capture to a new .npz file, then exit.")
    args = parser.parse_args()
    if args.record is not None and args.enable_control:
        parser.error("--record cannot be combined with --enable-control")
    cfg = Config(args.policy_config)
    config = yaml.safe_load(args.config.read_text())["robot_bridge"]["ros__parameters"]
    # Keep ROS alive until the Ctrl+C cleanup has requested a bridge stop.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("x2_policy_client")
    controller = None
    try:
        controller = RobotController(node, cfg, args, config)
        if args.record is None:
            rclpy.spin(node)
        else:
            waiting_since = time.monotonic()
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
                now = time.monotonic()
                if controller.record_started is not None:
                    if now - controller.record_started >= 10.0:
                        break
                elif now - waiting_since >= 30.0:
                    print("30 秒内没有有效推理帧，结束记录。", flush=True)
                    break
    except KeyboardInterrupt:
        if controller is not None:
            controller.exit_reason = "KeyboardInterrupt"
    except Exception as error:
        if controller is not None:
            controller.exit_reason = f"{type(error).__name__}: {error}"
        node.get_logger().error(str(error))
        raise SystemExit(1)
    finally:
        if controller is not None:
            try:
                controller.stop()
            except (RuntimeError, KeyboardInterrupt) as error:
                node.get_logger().error(f"Stop not confirmed: {error}; use robot-side takeover.")
            try:
                controller.save_control_recording()
                controller.save_recording()
            except OSError as error:
                print(f"记录保存失败：{error}", flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
