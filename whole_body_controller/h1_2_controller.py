"""H1-2 body client composition and DC4 Inspire hand communication."""

import math
from numbers import Real

from rclpy.qos import qos_profile_sensor_data
from unitree_go.msg import MotorCmd, MotorCmds, MotorStates

from robot_bridge_py.robot_client import RobotClient


class H1_2WholeBodyController:
    """Use the caller's ROS node and executor; construction sends no commands.

    body exposes the existing RobotClient unchanged. Its stop_control() only
    controls the body. Hand publication does not imply acceptance or completion.
    """

    def __init__(self, node, control_frequency=100, interpolation_order=0):
        self.body = RobotClient(
            node, "H1_2", 27, control_frequency,
            interpolation_order=interpolation_order,
        )
        self.hand_state = None
        # ROS names map to native DDS rt/inspire/cmd and rt/inspire/state.
        self._hand_publisher = node.create_publisher(MotorCmds, "/inspire/cmd", 1)
        self._hand_subscription = node.create_subscription(
            MotorStates, "/inspire/state", self._hand_state_callback,
            qos_profile_sensor_data,
        )

    def send_hand_cmd(self, *, left, right):
        """Publish both hands once, with six normalized positions per hand.

        Order: little, ring, middle, index, thumb bend, thumb rotation.
        Values are in [0, 1] (0 closed, 1 open), not radians.
        No interpolation, automatic repeat, or hand stop is implemented.
        """
        targets = {}
        for side, values in (("right", right), ("left", left)):
            try:
                values = list(values)
            except TypeError as exc:
                raise ValueError(f"{side} requires six numeric positions.") from exc
            if len(values) != 6 or any(
                isinstance(value, bool) or not isinstance(value, Real)
                or not math.isfinite(value) or not 0 <= value <= 1
                for value in values
            ):
                raise ValueError(f"{side} requires six finite positions in [0, 1].")
            targets[side] = [float(value) for value in values]

        message = MotorCmds()
        for value in targets["right"] + targets["left"]:
            command = MotorCmd()
            command.q = value
            message.cmds.append(command)
        self._hand_publisher.publish(message)

    def _hand_state_callback(self, message):
        # Keep the raw feedback, including lost counters; do not infer readiness.
        self.hand_state = message
