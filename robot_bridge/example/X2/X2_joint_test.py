import argparse
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
import yaml
from robot_bridge_py.robot_client import RobotClient


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
    def __init__(self, node, args, config):
        self.node = node
        self.args = args
        self.config = config
        self.num_dof = config["num_joint"]
        names = config["joint_names"]
        if len(names) != self.num_dof or [config[n]["idx"] for n in names] != list(range(self.num_dof)):
            raise ValueError("Joint names and indices must match the X2 feedback order.")
        self.index = config[args.joint]["idx"]
        self.kp = np.array([config[n]["kp"] for n in names])
        self.kd = np.array([config[n]["kd"] for n in names])
        self.q_min = np.array([config[n]["q_min"] for n in names])
        self.q_max = np.array([config[n]["q_max"] for n in names])
        self.robot = TestRobotClient(node, self.num_dof, config)
        self.delta = np.deg2rad(args.delta_deg)
        self.requested = False
        self.last_print = 0.0

    def snapshot(self):
        self.robot.check_state()
        q = self.robot.q_pos.copy()
        invalid = (q < self.q_min) | (q > self.q_max) | ~np.isfinite(q)
        if np.any(invalid):
            details = []
            for i in np.flatnonzero(invalid):
                actual, lower, upper = np.rad2deg([q[i], self.q_min[i], self.q_max[i]])
                excess = max(lower - actual, actual - upper, 0.0) if np.isfinite(actual) else float("nan")
                details.append(f"{self.config['joint_names'][i]}: actual={actual:+.3f} deg, "
                               f"limits=[{lower:+.3f}, {upper:+.3f}] deg, excess={excess:.3f} deg")
            raise ValueError("Starting positions outside configured limits:\n" + "\n".join(details) +
                             "\nNo commands sent by this check; limits are unchanged.")
        target = q[self.index] + self.delta
        if not self.q_min[self.index] <= target <= self.q_max[self.index]:
            raise ValueError("The selected trajectory exceeds configured position limits.")
        if abs(self.delta) * np.pi / self.args.duration > self.config[self.args.joint]["dq_limit"]:
            raise ValueError("The selected trajectory exceeds the configured velocity limit.")
        return q

    def step(self):
        self.robot.check_state()
        elapsed = time.monotonic() - self.started
        q = self.start_q.copy()
        dq = np.zeros(self.num_dof)
        q[self.index], dq[self.index] = joint_target(q[self.index], self.delta, elapsed, self.args.duration)
        self.robot.send_cmd(q, dq, target_kp=self.kp, target_kd=self.kd)
        if time.monotonic() - self.last_print >= 0.5:
            actual = self.robot.q_pos[self.index]
            other_error = np.delete(self.robot.q_pos - self.start_q, self.index)
            phase = "HOLD AT START" if elapsed >= self.args.duration else "MOVING"
            print(f"{phase}: target={q[self.index]:+.4f}, actual={actual:+.4f}, "
                  f"error={actual-q[self.index]:+.4f} rad; other max error={np.max(np.abs(other_error)):.4f} rad", flush=True)
            self.last_print = time.monotonic()

    def run(self):
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
        preview = self.snapshot()
        print(f"Joint: {self.args.joint}; start={preview[self.index]:.4f} rad; "
              f"peak={preview[self.index]+self.delta:.4f} rad; round trip={self.args.duration:.1f} s")
        print(f"All-joint holding gains from config: kp={self.kp.tolist()}, kd={self.kd.tolist()}")
        if not self.args.enable_control:
            print("READ-ONLY preview complete. No start request or commands sent.")
            return
        print("All joints will hold this posture. The selected joint will move once and return.")
        print("Afterwards this program keeps publishing position hold until Ctrl+C.")
        print("Ctrl+C only requests stop publishing; it does NOT provide damping or motor disable.")
        if input("Robot supported and robot-side stop ready? Type MOVE to start: ").strip() != "MOVE":
            return
        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.start_q = self.snapshot()
        if np.max(np.abs(self.start_q - preview)) > 0.02:
            raise RuntimeError("Posture changed since preview. Run again to review the new posture.")
        if self.node.count_publishers('/robot_cmd') != 1:
            raise RuntimeError("Another /robot_cmd publisher exists. Close other command clients first.")
        if not self.robot.start_control_client.service_is_ready() or not self.robot.stop_control_client.service_is_ready():
            raise RuntimeError("Bridge start/stop services are unavailable.")
        self.requested = True
        future = self.robot.init_control(default_pos=self.start_q)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        if not future.done() or future.exception() or not future.result().success:
            raise RuntimeError("Start was not confirmed. Verify the robot; a timed-out request may still execute.")
        deadline = time.monotonic() + self.config["duration"] + 0.2
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            self.robot.check_state()
        self.started = time.monotonic()
        self.timer = self.node.create_timer(0.02, self.step)
        rclpy.spin(self.node)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Preview or test one X2 joint while all others hold their starting positions.")
    parser.add_argument("--config", type=Path, default=Path(__file__).resolve().parents[2] / "params/X2_config.yaml")
    parser.add_argument("--joint", required=True, help="Exact joint name in X2_config.yaml.")
    parser.add_argument("--delta-deg", type=float, required=True, help="Signed peak displacement in degrees, at most 5 degrees.")
    parser.add_argument("--duration", type=float, default=6.0, help="Total outward-and-return time in seconds, at least 2.")
    parser.add_argument("--enable-control", action="store_true", help="Allow a start request and whole-body position-hold commands after confirmation.")
    args = parser.parse_args()
    if not np.isfinite(args.delta_deg) or not 0.0 < abs(args.delta_deg) <= 5.0:
        parser.error("--delta-deg must be finite, nonzero and within [-5, 5]. This cap is not a validated safe amplitude.")
    if not np.isfinite(args.duration) or args.duration < 2.0:
        parser.error("--duration must be finite and at least 2 seconds.")
    with open(args.config) as file:
        config = yaml.safe_load(file)["robot_bridge"]["ros__parameters"]
    if args.joint not in config["joint_names"]:
        parser.error("Unknown joint. Choose from: " + ", ".join(config["joint_names"]))
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("x2_joint_test")
    controller = None
    try:
        controller = RobotController(node, args, config)
        controller.run()
    except KeyboardInterrupt:
        print("Interrupted. Use the confirmed robot-side stop or takeover procedure.")
    except (RuntimeError, ValueError) as error:
        print(f"TEST ABORTED: {error}")
    finally:
        if controller is not None and controller.requested:
            if hasattr(controller, "timer"):
                controller.timer.cancel()
            try:
                if not rclpy.ok():
                    raise RuntimeError("ROS context is shut down; stop request cannot be sent.")
                future = controller.robot.stop_control()
                rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
                if not future.done() or future.exception() or not future.result().success:
                    raise RuntimeError("Stop publishing was not confirmed.")
                print("Bridge acknowledged stop publishing. This does not confirm a safe robot stop.")
            except Exception as error:
                print(f"STOP REQUEST FAILED: {error}. Use the robot-side stop.")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
