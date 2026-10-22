import time
import numpy as np
from robot_bridge.robot_client import RobotClient


class PolicyRobotClient(RobotClient):
    def __init__(self, node, config, areas):
        self.config = config
        self.areas = areas
        self.received = {}
        self.head_pitch = np.zeros(2, dtype=np.float32)
        super().__init__(node, "X2", 30, 50.0, interpolation_order=0.0, enable_joystick=False)

    def _check_joint_state_message_x2(self, msg, message_count, active_count, group_name):
        names = self.areas[group_name]["joints"]
        valid = super()._check_joint_state_message_x2(msg, message_count, len(names), group_name)
        valid = valid and [j.name for j in msg.joints] == names
        self.received[group_name] = time.monotonic() if valid else 0.0
        if valid and group_name == "head":
            self.head_pitch[:] = [msg.joints[1].position, msg.joints[1].velocity]
        return valid

    def _imu_state_handler_x2(self, msg):
        self._quat[:] = 0.0
        super()._imu_state_handler_x2(msg)
        self.received["imu"] = time.monotonic() if np.linalg.norm(self._quat) > 0 else 0.0

    def unavailable_groups(self):
        now = time.monotonic()
        return [g for g in (*self.areas, "imu")
                if now - self.received.get(g, 0.0) > self.config[f"{g}_state_timeout"]]
