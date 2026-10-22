import time
from collections import deque
from datetime import datetime
import hashlib
import json
from pathlib import Path

import numpy as np


class ControlLog:
    def __init__(self, cfg, config):
        self.frames = deque(maxlen=251)
        self.metadata = {
            "model_sha256": hashlib.sha256(cfg.policy_path.read_bytes()).hexdigest(),
            "bridge_config": config,
            "policy_config": cfg.data,
            "joint_names": config["joint_names"], "obs_joint_names": cfg.obs_joint_names,
            "action_joint_names": cfg.action_joint_names,
            "units": "rad, rad/s; quaternion wxyz; monotonic seconds",
            "note": "sent means client send_cmd returned, not hardware acknowledgement; NaN means not computed."}

    def capture(self, controller, phase, target):
        now = time.monotonic()
        frame = {"time": now, "phase": phase, "step": controller.counter, "sent": False,
                 "q": controller.robot.q_pos.copy(), "dq": controller.robot.q_vel.copy(),
                 "head_pitch": controller.robot.head_pitch.copy(), "quat": controller.robot.quat.copy(),
                 "angular_velocity": controller.robot.angular_velocity.copy(),
                 "velocity_cmd": controller.control.velocity_cmd.copy(), "target_q": target.copy(),
                 "previous_action": controller.last_action.copy(), "obs": np.full(500, np.nan),
                 "raw_action": np.full(29, np.nan), "action": np.full(29, np.nan),
                 "state_age": np.array([now - controller.robot.received[g]
                                        for g in (*controller.policy_config.joint_areas, "imu")])}
        self.frames.append(frame)
        while self.frames and now - self.frames[0]["time"] > 5.0:
            self.frames.popleft()
        return frame

    def save(self, controller):
        if not self.frames:
            return
        folder = Path.home() / "x2_logs"
        folder.mkdir(exist_ok=True)
        path = folder / ("policy_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".npz")
        arrays = {key: np.stack([frame[key] for frame in self.frames])
                  for key in self.frames[0]}
        metadata = dict(self.metadata, exit_reason=controller.exit_reason,
                        sample_count=len(self.frames),
                        state_age_groups=[*controller.policy_config.joint_areas, "imu"])
        with path.open("xb") as stream:
            np.savez_compressed(stream, metadata=json.dumps(metadata), **arrays)
        print(f"控制记录已保存 {len(self.frames)} 帧：{path}", flush=True)

    def report(self, controller, now, elapsed_ms):
        if now - controller.control.last_print >= 1.0:
            outside = [controller.names[i] for i in controller.action_indices if not controller.q_min[i] <= controller.target_q[i] <= controller.q_max[i]]
            print(f"控制 #{controller.counter} | 速度 {controller.control.velocity_cmd.round(2)} | "
                  f"推理 {elapsed_ms:.1f} ms | 目标超出配置范围 {len(outside)}", flush=True)
            for name in outside:
                i = controller.names.index(name)
                actual, target, lower, upper = np.rad2deg(
                    [controller.robot.q_pos[i], controller.target_q[i], controller.q_min[i], controller.q_max[i]])
                excess = max(lower - target, target - upper, 0.0)
                print(f"目标范围提示：{name}\n"
                      f"  实测 {actual:+.3f}° | 模型目标 {target:+.3f}°\n"
                      f"  配置角度范围 [{lower:+.3f}°, {upper:+.3f}°] | 目标超出 {excess:.3f}°", flush=True)
            controller.control.last_print = now
