import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import Joy
from robot_bridge.robot_client import RemoteController, KeyMap


def load_bridge_config():
    path = Path(__file__).resolve().parents[3] / "params/X2_config.yaml"
    return yaml.safe_load(path.read_text())["robot_bridge"]["ros__parameters"]


class ControlSession:
    """Bridge requests, joystick events and preparation for a policy controller."""

    def __init__(self, controller):
        self.controller = controller
        node = controller.node
        config = controller.config
        self.init_pos = np.asarray(config["ready_q"], dtype=np.float64)
        self.prep_kp = np.array([config[n]["kp"] for n in self.controller.names])
        self.prep_kd = np.array([config[n]["kd"] for n in self.controller.names])
        self.controller.robot._default_duration = float(config["duration"])
        self.controller.robot.set_default_cmd(self.init_pos, self.prep_kp, self.prep_kd)
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
        self.joy_subscriber = node.create_subscription(Joy, "/joy", self.joy_callback, qos_profile_sensor_data)
        self.was_ready = False
        self.last_print = 0.0

    def announce(self):
        print(f"X2 policy：{self.controller.policy_config.policy_path}")
        print("控制模式：先启动 X2 Bridge 和 game_controller_node。")
        print("L2 + START：准备姿态；R2 + ×：过渡到模型姿态后自动开启 policy。")
        print("Policy 目标超过配置角度范围仅提示；Bridge 保留力矩约束。")
        print("摇杆控制速度；L2 + UP + LEFT：停止发送并请求 Bridge 停止。")
        print("新版 X2 Bridge 收到停止请求后会锁定阻尼；不会退出调试模式。恢复控制需重启 Bridge。")
        print("50 Hz policy；头 pitch 必须有有效反馈。")
        self.report_gain_limits()

    def report_gain_limits(self):
        clipped = []
        for i in range(30):
            prefix = "lower_limbs" if self.controller.config[self.controller.names[i]]["if_strong_joint"] else "upper_limbs"
            if any(not self.controller.config[f"{prefix}_{key}_min"] <= value <= self.controller.config[f"{prefix}_{key}_max"]
                   for key, value in (("kp", self.controller.kp[i]), ("kd", self.controller.kd[i]))):
                clipped.append(self.controller.names[i])
        if clipped:
            message = "Policy gains exceed bridge configuration: " + ", ".join(clipped)
            raise ValueError(message)
        self.check_target(self.init_pos)
        self.check_target(self.controller.default_q)

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
        outside = [self.controller.names[i] for i in range(30) if not self.controller.q_min[i] <= target[i] <= self.controller.q_max[i]]
        if outside:
            details = []
            for name in outside:
                i = self.controller.names.index(name)
                actual, desired, lower, upper = np.rad2deg(
                    [self.controller.robot.q_pos[i], target[i], self.controller.q_min[i], self.controller.q_max[i]])
                details.append(f"{name}: actual={actual:+.3f} deg, target={desired:+.3f} deg, "
                               f"limits=[{lower:+.3f}, {upper:+.3f}] deg, "
                               f"excess={max(lower - desired, desired - upper):.3f} deg")
            raise RuntimeError("Joint target outside configured limits: " + "; ".join(details))

    def request_stop(self):
        self.agent_started = False
        self.transition_start = None
        self.ready_at = None
        self.controller.robot.control_started = False
        self.controller.robot.control_start_time = None
        self.velocity_cmd[:] = 0.0
        self.joy_key = None
        self.stop_requested = True
        # Let a pending start finish before sending stop, so start cannot overtake it.
        if self.control_requested and self.pending is None:
            self.pending = self.controller.robot.stop_control()
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
                if actual != [self.controller.config[name] for name in self.parameter_names]:
                    raise RuntimeError("Bridge parameters differ; restart Bridge with the same X2_config.yaml.")
                if self.controller.robot.unavailable_groups() or self.controller.node.count_publishers("/robot_cmd") != 1:
                    raise RuntimeError("Feedback or command ownership changed before start.")
                self.check_target(self.controller.robot.q_pos)
                self.control_requested = True
                self.pending = self.controller.robot.init_control(self.init_pos)
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
                self.ready_at = now + self.controller.robot._default_duration
        if self.ready_at is not None and now >= self.ready_at:
            self.ready_at = None
            self.controller.robot.control_started = True
            print("准备姿态阶段完成；按 R2 + × 过渡到模型姿态，随后自动开启 policy。", flush=True)
        buttons, self.joy_key = self.joy_key, None
        if buttons is None or now - self.joy_time > 0.5:
            return
        combo = self.remote.is_exact_combo
        if combo(buttons, [KeyMap.L2, KeyMap.up, KeyMap.left]):
            self.request_stop()
        elif combo(buttons, [KeyMap.L2, KeyMap.start]) and not self.control_requested:
            if self.controller.robot.unavailable_groups():
                self.controller.node.get_logger().warn("Waiting for valid robot feedback.")
                return
            self.check_target(self.controller.robot.q_pos)
            if self.controller.node.count_publishers("/robot_cmd") != 1:
                raise RuntimeError("Close other /robot_cmd clients before starting.")
            if not self.controller.robot.stop_control_client.service_is_ready():
                raise RuntimeError("Bridge stop service is unavailable.")
            self.stop_requested = False
            if not self.bridge_parameters.service_is_ready():
                raise RuntimeError("Bridge parameter service is unavailable.")
            self.pending = self.bridge_parameters.call_async(GetParameters.Request(names=self.parameter_names))
            self.request_kind = "parameters"
            self.request_time = now
        elif combo(buttons, [KeyMap.R2, KeyMap.A]) and self.controller.robot.control_started:
            if self.agent_started or self.transition_start is not None:
                return
            if np.any(np.abs(self.velocity_cmd) > 0.02):
                self.controller.node.get_logger().warn("Center the sticks before starting policy.")
                return
            self.transition_q = self.controller.robot.q_pos.copy()
            self.transition_start = now
            self.controller.reset_history()
            self.velocity_cmd[:] = 0.0
            print("正在过渡到模型默认姿态，1.5 秒后自动开启 policy。", flush=True)

    def update(self, now):
        if now - self.joy_time > 0.5:
            self.velocity_cmd[:] = 0.0
        # A stop edge takes priority even while a start request is pending.
        if self.joy_key is not None and self.remote.is_exact_combo(
                self.joy_key, [KeyMap.L2, KeyMap.up, KeyMap.left]):
            self.request_stop()
        self.check_state()
        missing = self.controller.robot.unavailable_groups()
        if missing:
            if self.control_requested:
                raise RuntimeError("Robot feedback unavailable: " + ", ".join(missing))
            if self.was_ready:
                self.controller.reset_history()
            self.was_ready = False
            if now - self.last_print >= 1.0:
                print("等待有效反馈：" + ", ".join(missing), flush=True)
                self.last_print = now
            return False
        self.was_ready = True
        if not self.controller.robot.control_started or self.stop_requested:
            return False
        if self.controller.node.count_publishers("/robot_cmd") != 1:
            raise RuntimeError("Another /robot_cmd publisher appeared.")
        return True

    def send_preparation_command(self, now):
        if self.transition_start is not None:
            phase = min((now - self.transition_start) / 1.5, 1.0)
            weight = 0.5 * (1.0 - np.cos(np.pi * phase))
            target = self.transition_q + weight * (self.controller.default_q - self.transition_q)
            frame = self.controller.recorder.capture(self.controller, "transition", target)
            self.check_target(target)
            self.controller.robot.send_cmd(target, target_kp=self.controller.kp, target_kd=self.controller.kd)
            frame["sent"] = True
            if phase >= 1.0:
                self.transition_start = None
                self.agent_started = True
                self.controller.reset_history()
                self.velocity_cmd[:] = 0.0
                print("Policy 已开启：历史已重置，前 5 帧暖机；摇杆控制速度。", flush=True)
            return True
        if not self.agent_started:
            frame = self.controller.recorder.capture(self.controller, "preparation_hold", self.init_pos)
            self.controller.robot.send_cmd(self.init_pos, target_kp=self.prep_kp, target_kd=self.prep_kd)
            frame["sent"] = True
            return True
        return False

    def stop(self):
        self.controller.timer.cancel()
        if not self.control_requested:
            return
        self.request_stop()
        deadline = time.monotonic() + 2.0
        while self.control_requested and time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self.controller.node, timeout_sec=0.05)
            self.check_state()
        if self.control_requested:
            self.controller.node.get_logger().error("Stop not confirmed; use robot-side stop or takeover.")


def run_controller(controller_type, config):
    # Keep ROS alive until the Ctrl+C cleanup has requested a bridge stop.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("x2_policy_client")
    controller = None
    try:
        controller = controller_type(node, config)
        rclpy.spin(node)
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
                controller.control.stop()
            except (RuntimeError, KeyboardInterrupt) as error:
                node.get_logger().error(f"Stop not confirmed: {error}; use robot-side takeover.")
            try:
                controller.recorder.save(controller)
            except OSError as error:
                print(f"记录保存失败：{error}", flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
