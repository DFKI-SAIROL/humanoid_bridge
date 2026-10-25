"""Fake Booster T1: /low_state at 500 Hz, its gamepad at 50 Hz, booster_rpc_service; no /joint_ctrl.

LowState carries SERIAL_MOTORS serial motor states, more than T1_config.yaml's joints: the bridge
must bound its copy. Test inputs: /fake_t1/buttons (String, held buttons as `RB+RT`, "" releases),
/fake_t1/remote (Bool, gamepad messages on/off). Out: /fake_t1/mode (String, each mode request's
body, transient-local), and /joint_ctrl tracked by the first NUM_JOINT motors.
"""

import argparse
import signal
import sys

import rclpy
from booster_interface.msg import LowCmd, LowState, MotorState, RemoteControllerState
from booster_interface.srv import RpcService
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

NUM_JOINT = 23  # T1_config.yaml
SERIAL_MOTORS = 25  # 2 more than configured: the old handler wrote past currentState_
STATE_PERIOD = 0.002
PAD_PERIOD = 0.02
RPY = [0.1, 0.0, 0.0]  # drive_t1.py finds the quaternion again on /robot_state
BUTTONS = ("a", "b", "x", "y", "lb", "rb", "lt", "rt", "back", "start", "ls", "rs")
CHANGE_MODE = 2000  # LocoApiId::kChangeMode


class FakeT1(Node):
    def __init__(self, start_q):
        super().__init__("fake_t1")
        self.state = LowState(motor_state_serial=[MotorState() for _ in range(SERIAL_MOTORS)])
        self.state.imu_state.rpy = RPY
        self.state.imu_state.acc = [0.0, 0.0, 9.81]
        self.q = [float(start_q)] * SERIAL_MOTORS
        self.held = set()
        self.remote_on = True
        self.publisher = self.create_publisher(LowState, "/low_state", 10)
        self.pad = self.create_publisher(RemoteControllerState, "/remote_controller_state", 10)
        latched = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)  # fmt: skip
        self.modes = self.create_publisher(String, "/fake_t1/mode", latched)
        self.create_service(RpcService, "booster_rpc_service", self._on_rpc)
        self.create_subscription(LowCmd, "/joint_ctrl", self._on_cmd, 10)
        self.create_subscription(String, "/fake_t1/buttons", self._on_buttons, 10)
        self.create_subscription(Bool, "/fake_t1/remote", self._on_remote, 10)
        self.create_timer(STATE_PERIOD, self._tick)
        self.create_timer(PAD_PERIOD, self._pad)

    def _tick(self):
        for i, m in enumerate(self.state.motor_state_serial):
            m.q = self.q[i]
        self.publisher.publish(self.state)

    def _pad(self):
        if not self.remote_on:
            return
        self.pad.publish(RemoteControllerState(**{b: b in self.held for b in BUTTONS}))

    def _on_buttons(self, msg):
        self.held = {b.strip().lower() for b in msg.data.split("+") if b.strip()}

    def _on_remote(self, msg):
        self.remote_on = bool(msg.data)

    def _on_cmd(self, msg):
        for i in range(min(len(msg.motor_cmd), NUM_JOINT)):
            if msg.motor_cmd[i].kp > 0:
                self.q[i] = msg.motor_cmd[i].q  # perfect tracking

    def _on_rpc(self, request, response):
        if request.msg.api_id == CHANGE_MODE:
            self.modes.publish(String(data=request.msg.body))
            print(f"fake_t1: change mode {request.msg.body}", flush=True)
        response.msg.status = 0
        return response


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--start-q", type=float, default=0.0)
    args = parser.parse_args()
    rclpy.init()
    node = FakeT1(args.start_q)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
