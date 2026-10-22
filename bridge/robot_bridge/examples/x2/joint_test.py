import argparse
import select
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
import yaml
from robot_bridge.robot_client import RobotClient


class TestRobotClient(RobotClient):
    def __init__(self, node, num_dof, config):
        self.received = {}
        self.config = config
        super().__init__(node, "X2", num_dof, 50.0, interpolation_order=0.0)

    def _check_joint_state_message_x2(self, msg, message_count, active_count, group_name):
        valid = super()._check_joint_state_message_x2(msg, message_count, active_count, group_name)
        offset = {"leg": 0, "waist": 12, "arm": 15, "head": 29}[group_name]
        names = self.config["joint_names"][offset:offset + active_count]
        valid = valid and [j.name for j in msg.joints[:active_count]] == names
        self.received[group_name] = time.monotonic() if valid else 0.0
        return valid

    def _imu_state_handler_x2(self, msg):
        self._quat[:] = 0.0
        super()._imu_state_handler_x2(msg)
        self.received["imu"] = time.monotonic() if np.linalg.norm(self._quat) > 0.0 else 0.0

    def check_state(self):
        groups = ["leg", "waist", "arm", "imu"] + (["head"] if self.num_dof == 30 else [])
        for group in groups:
            if time.monotonic() - self.received.get(group, 0.0) > self.config[f"{group}_state_timeout"]:
                raise RuntimeError(f"Missing, invalid or stale {group} feedback. Use the robot-side stop if controlling.")


def joint_target(start, delta, elapsed, duration):
    # One smooth outward-and-return cycle, then hold the starting position.
    if elapsed >= duration:
        return start, 0.0
    phase = 2.0 * np.pi * max(0.0, elapsed) / duration
    return start + delta * (1.0 - np.cos(phase)) / 2.0, delta * np.pi * np.sin(phase) / duration


class RobotController:
    def __init__(self, node, config):
        self.node = node
        self.config = config
        self.names = config["joint_names"]
        self.num_dof = config["num_joint"]
        if len(self.names) != self.num_dof or [config[n]["idx"] for n in self.names] != list(range(self.num_dof)):
            raise ValueError("Joint names and indices must match the X2 feedback order.")
        self.kp, self.kd, self.q_min, self.q_max = (
            np.array([config[n][key] for n in self.names]) for key in ("kp", "kd", "q_min", "q_max"))
        self.robot = TestRobotClient(node, self.num_dof, config)
        self.requested = False
        self.timer = None
        self.start_q = None
        self.moving = False

    def prompt(self, text):
        # Keep ROS callbacks and position holding running while waiting for input.
        print(text, end="", flush=True)
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline()
                if not line or line.strip().lower() == "q":
                    raise EOFError
                return line.strip()
        raise RuntimeError("ROS stopped.")

    def snapshot(self):
        self.robot.check_state()
        q = self.robot.q_pos.copy()
        invalid = (q < self.q_min) | (q > self.q_max) | ~np.isfinite(q)
        if np.any(invalid):
            details = [f"{self.names[i]}: {np.rad2deg(q[i]):+.2f} deg, "
                       f"limits [{np.rad2deg(self.q_min[i]):+.2f}, {np.rad2deg(self.q_max[i]):+.2f}]"
                       for i in np.flatnonzero(invalid)]
            raise RuntimeError("Invalid starting posture:\n" + "\n".join(details))
        return q

    def wait_reply(self, future, timeout):
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        if not future.done() or future.exception() or not future.result().success:
            raise RuntimeError("Bridge request was not confirmed; check robot-side control.")

    def start(self, posture):
        current = self.snapshot()
        if np.max(np.abs(current - posture)) > 0.02:
            raise RuntimeError("Posture changed during confirmation. Restart to capture the new posture.")
        if self.node.count_publishers("/robot_cmd") != 1:
            raise RuntimeError("Another /robot_cmd publisher exists. Close the other control client.")
        if not self.robot.stop_control_client.service_is_ready():
            raise RuntimeError("Bridge stop service is unavailable.")
        self.start_q = posture.copy()
        self.requested = True
        self.wait_reply(self.robot.init_control(default_pos=self.start_q), 5.0)
        deadline = time.monotonic() + self.config["duration"] + 0.2
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            self.robot.check_state()
        self.timer = self.node.create_timer(0.02, self.step)

    def step(self):
        self.robot.check_state()
        q = self.start_q.copy()
        dq = np.zeros(self.num_dof)
        if self.moving:
            elapsed = time.monotonic() - self.started
            q[self.index], dq[self.index] = joint_target(q[self.index], self.delta, elapsed, self.duration)
            actual = self.robot.q_pos[self.index]
            self.travel = max(self.travel, abs(actual - self.actual_start))
            self.error = max(self.error, abs(actual - q[self.index]))
            # Allow a short settling period before reporting the return error.
            if elapsed >= self.duration + 0.5:
                self.moving = False
                print(f"完成 {self.names[self.index]}：实际最大位移 {np.rad2deg(self.travel):.2f}°，"
                      f"最大跟踪误差 {np.rad2deg(self.error):.2f}°，"
                      f"回位误差 {np.rad2deg(abs(actual - self.start_q[self.index])):.2f}°")
                print("正在保持起始姿态，可继续选择关节。", flush=True)
        self.robot.send_cmd(q, dq, target_kp=self.kp, target_kd=self.kd)

    def run(self):
        print("X2 交互测试：每次只动一个关节，其他关节保持首次确认的姿态。")
        print("等待输入时继续保持；输入 q 或 Ctrl+C 请求停止并退出；新版 X2 Bridge 将锁定阻尼，不会退出调试模式。")
        for i, name in enumerate(self.names, 1):
            print(f"{i:2d}  {name}")
        while True:
            try:
                index = int(self.prompt("\n关节编号（q 退出）：")) - 1
                if not 0 <= index < self.num_dof:
                    raise ValueError("关节编号不在列表内。")
                degrees = float(self.prompt("运动角度（度，默认 2，可填负数）：") or "2")
                duration = float(self.prompt("往返时间（秒，默认 6）：") or "6")
                if not np.isfinite(degrees) or not 0 < abs(degrees) <= 5:
                    raise ValueError("角度须非零，范围 -5° 到 +5°。")
                if not np.isfinite(duration) or duration < 2:
                    raise ValueError("往返时间须至少 2 秒。")
                posture = self.snapshot() if self.start_q is None else self.start_q
                delta = np.deg2rad(degrees)
                if not self.q_min[index] <= posture[index] + delta <= self.q_max[index]:
                    raise ValueError("目标角度超出配置限位，请重新选择。")
                if abs(delta) * np.pi / duration > self.config[self.names[index]]["dq_limit"]:
                    raise ValueError("目标速度超限，请增加往返时间。")
                print(f"{self.names[index]}：{np.rad2deg(posture[index]):+.2f}° → "
                      f"{np.rad2deg(posture[index] + delta):+.2f}° → 起点，往返 {duration:g} 秒。")
                if self.prompt("回车开始（q 退出）："):
                    print("本次取消。")
                    continue
                if self.start_q is None:
                    self.start(posture)
                self.robot.check_state()
                self.index, self.delta, self.duration = index, delta, duration
                self.actual_start = self.robot.q_pos[index]
                self.travel = self.error = 0.0
                self.started = time.monotonic()
                self.moving = True
                while self.moving:
                    rclpy.spin_once(self.node, timeout_sec=0.02)
            except ValueError as error:
                print(f"输入无效：{error}")

    def close(self):
        if self.timer is not None:
            self.timer.cancel()
        if self.requested:
            try:
                self.wait_reply(self.robot.stop_control(), 2.0)
                print("Bridge 停止请求已确认；新版 X2 Bridge 将持续发阻尼，恢复控制需重启 Bridge。")
            except Exception as error:
                print(f"停止请求未确认：{error}，请使用机器人端停止方式。")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Interactively test X2 joints while holding the initial posture.")
    parser.add_argument("--config", type=Path, default=Path(__file__).resolve().parents[2] / "params/X2_config.yaml")
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())["robot_bridge"]["ros__parameters"]
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("x2_joint_test")
    controller = None
    try:
        controller = RobotController(node, config)
        controller.run()
    except (KeyboardInterrupt, EOFError):
        print("\n退出测试。")
    except RuntimeError as error:
        print(f"测试中止：{error}")
    finally:
        if controller is not None:
            controller.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
