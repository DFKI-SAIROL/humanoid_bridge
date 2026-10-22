"""Command-only client for /robot_cmd and /hand_cmd/*: publishers plus start/stop.

No state subscriptions; see robot_client.RobotClient for those.
"""

import numpy as np
import rclpy
from bridge_interface.msg import HandCmd, MotorCmd, RobotCmd
from bridge_interface.srv import SetDefaultPosition
from std_srvs.srv import Trigger

TOPIC = "/robot_cmd"

# The bridge overwrites mode on every motor; 1 matches what it writes.
MOTOR_MODE = 1

HAND_TOPIC = "/hand_cmd/{side}"
HAND_SIDES = ("left", "right")

# Dex3-1, one hand. Order is DDS order:
# thumb_0 thumb_1 thumb_2 middle_0 middle_1 index_0 index_1.
NUM_HAND_JOINTS = 7

# GEAR's goal order groups the fingers the other way round. Anything coming from
# a policy goes through robot_bridge.adapters.gear_hands, not straight in here.

# The bridge packs the Dex3 mode bitfield itself, per motor index, so a client
# cannot write a useful value. Left at 0 and overwritten.
HAND_MOTOR_MODE = 0


def pack_robot_cmd(
    q, dq, tau, kp, kd, interpolation_order=1.0, duration=0.02, hold_position=False, mode=MOTOR_MODE
):
    """Build a RobotCmd from five equal-length per-motor arrays."""
    lengths = {len(a) for a in (q, dq, tau, kp, kd)}
    if len(lengths) != 1:
        raise ValueError(
            "q, dq, tau, kp, kd must have equal length, got "
            f"{[len(a) for a in (q, dq, tau, kp, kd)]}"
        )

    cmd = RobotCmd()
    cmd.interpolation_order = float(interpolation_order)
    cmd.hold_position = bool(hold_position)
    cmd.duration = float(duration)

    motor_cmd = []
    for i in range(lengths.pop()):
        motor = MotorCmd()
        motor.mode = mode
        motor.q = float(q[i])
        motor.dq = float(dq[i])
        motor.tau = float(tau[i])
        motor.kp = float(kp[i])
        motor.kd = float(kd[i])
        motor_cmd.append(motor)
    cmd.motor_cmd = motor_cmd
    return cmd


def pack_hand_cmd(q, dq, tau, kp, kd, duration=0.01, hold_position=False):
    """Build a HandCmd for one hand from five equal-length per-motor arrays."""
    lengths = {len(a) for a in (q, dq, tau, kp, kd)}
    if len(lengths) != 1:
        raise ValueError(
            "q, dq, tau, kp, kd must have equal length, got "
            f"{[len(a) for a in (q, dq, tau, kp, kd)]}"
        )

    cmd = HandCmd()
    cmd.hold_position = bool(hold_position)
    cmd.duration = float(duration)

    motor_cmd = []
    for i in range(lengths.pop()):
        motor = MotorCmd()
        motor.mode = HAND_MOTOR_MODE
        motor.q = float(q[i])
        motor.dq = float(dq[i])
        motor.tau = float(tau[i])
        motor.kp = float(kp[i])
        motor.kd = float(kd[i])
        motor_cmd.append(motor)
    cmd.motor_cmd = motor_cmd
    return cmd


class RobotCmdClient:
    """RobotClient's command surface without the /lowstate and joystick subscriptions.

    Service calls spin the node; do not use them while another thread spins it.
    """

    def __init__(
        self,
        node,
        num_dof,
        control_frequency,
        interpolation_order=1.0,
        topic=TOPIC,
        default_pos=None,
        default_kp=None,
        default_kd=None,
        hand_control_frequency=100.0,
    ):
        self.node = node
        self.num_dof = num_dof
        self.control_frequency = control_frequency
        self.interpolation_order = interpolation_order
        self.duration = 1.0 / control_frequency

        self._default_pos = self._as_array(default_pos, "default_pos")
        self._default_kp = self._as_array(default_kp, "default_kp")
        self._default_kd = self._as_array(default_kd, "default_kd")
        self._zeros = np.zeros(num_dof, dtype=np.float32)

        self.cmd_publisher = node.create_publisher(RobotCmd, topic, 1)
        self.start_control_client = node.create_client(SetDefaultPosition, "/start_control")
        self.stop_control_client = node.create_client(Trigger, "/stop_control")
        self.ready_position_control_client = node.create_client(Trigger, "/ready_position_control")
        self.estop_damp_client = node.create_client(Trigger, "/estop_damp")
        self.estop_hold_client = node.create_client(Trigger, "/estop_hold")
        self.estop_reset_client = node.create_client(Trigger, "/estop_reset")

        # Hands are a separate path: their own topics, their own enable, their own
        # rate. Nothing here is sized by num_dof.
        self.hand_duration = 1.0 / hand_control_frequency
        self._hand_zeros = np.zeros(NUM_HAND_JOINTS, dtype=np.float32)
        self._default_hand_kp = None
        self._default_hand_kd = None

        self.hand_cmd_publishers = {
            side: node.create_publisher(HandCmd, HAND_TOPIC.format(side=side), 1)
            for side in HAND_SIDES
        }
        self.start_hand_control_client = node.create_client(Trigger, "/start_hand_control")
        self.stop_hand_control_client = node.create_client(Trigger, "/stop_hand_control")

    def _as_array(self, value, name):
        if value is None:
            return None
        arr = np.asarray(value, dtype=np.float32)
        if arr.shape != (self.num_dof,):
            raise ValueError(f"{name} must have length {self.num_dof}, got {arr.shape}")
        return arr

    def _resolve(self, value, fallback, name, setter="set_default_cmd"):
        if value is not None:
            return value
        if fallback is None:
            raise ValueError(
                f"{name} is None and no default was set; pass {name} or set it via {setter}()"
            )
        return fallback

    def _call(self, client, request, timeout_sec):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(f"{client.srv_name} is not available; is the bridge running?")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
        if not future.done():
            raise RuntimeError(f"{client.srv_name} did not respond within {timeout_sec}s")
        return future.result()

    def set_default_cmd(self, default_pos=None, default_kp=None, default_kd=None):
        """Set what send_cmd falls back to when an argument is None."""
        if default_pos is not None:
            self._default_pos = self._as_array(default_pos, "default_pos")
        if default_kp is not None:
            self._default_kp = self._as_array(default_kp, "default_kp")
        if default_kd is not None:
            self._default_kd = self._as_array(default_kd, "default_kd")

    def send_cmd(
        self,
        q_target_pos=None,
        q_target_vel=None,
        target_tau=None,
        target_kp=None,
        target_kd=None,
        duration=None,
        hold_position=False,
    ):
        """Publish one /robot_cmd. Dropped by the bridge until start_control succeeds."""
        cmd = pack_robot_cmd(
            q=self._resolve(q_target_pos, self._default_pos, "q_target_pos"),
            dq=q_target_vel if q_target_vel is not None else self._zeros,
            tau=target_tau if target_tau is not None else self._zeros,
            kp=self._resolve(target_kp, self._default_kp, "target_kp"),
            kd=self._resolve(target_kd, self._default_kd, "target_kd"),
            interpolation_order=self.interpolation_order,
            duration=self.duration if duration is None else duration,
            hold_position=hold_position,
        )
        if len(cmd.motor_cmd) != self.num_dof:
            raise ValueError(f"expected {self.num_dof} motors, packed {len(cmd.motor_cmd)}")
        self.cmd_publisher.publish(cmd)
        return cmd

    def start_control(self, default_position=None, timeout_sec=5.0):
        """Ramp to default_position, then accept /robot_cmd. Blocks on the bridge."""
        request = SetDefaultPosition.Request()
        request.default_position = [
            float(v) for v in self._resolve(default_position, self._default_pos, "default_position")
        ]
        return self._call(self.start_control_client, request, timeout_sec)

    def stop_control(self, timeout_sec=5.0):
        """Hold the pose and drop /robot_cmd until start_control. Blocks on the bridge."""
        return self._call(self.stop_control_client, Trigger.Request(), timeout_sec)

    def ready_position_control(self, timeout_sec=5.0):
        """Ramp every joint to the config's ready_q; arms first if needed. Blocks on the bridge."""
        return self._call(self.ready_position_control_client, Trigger.Request(), timeout_sec)

    def estop_damp(self, timeout_sec=5.0):
        """Latched e-stop, kp 0: arms sink slowly. Arming refused until estop_reset."""
        return self._call(self.estop_damp_client, Trigger.Request(), timeout_sec)

    def estop_hold(self, timeout_sec=5.0):
        """Latched e-stop, hold the measured pose. Arming refused until estop_reset."""
        return self._call(self.estop_hold_client, Trigger.Request(), timeout_sec)

    def estop_reset(self, timeout_sec=5.0):
        """Clear the latch; success is false while the cause is still present."""
        return self._call(self.estop_reset_client, Trigger.Request(), timeout_sec)

    # --- Dex3 hands --------------------------------------------------------

    def _as_hand_array(self, value, name):
        if value is None:
            return None
        arr = np.asarray(value, dtype=np.float32)
        if arr.shape != (NUM_HAND_JOINTS,):
            raise ValueError(f"{name} must have length {NUM_HAND_JOINTS}, got {arr.shape}")
        return arr

    def set_default_hand_cmd(self, default_kp=None, default_kd=None):
        """Set what send_hand_cmd falls back to when kp or kd is None.

        There are no baked-in hand gains, for the same reason the body path has
        none: gains that arrive by default are gains nobody chose.
        """
        if default_kp is not None:
            self._default_hand_kp = self._as_hand_array(default_kp, "default_kp")
        if default_kd is not None:
            self._default_hand_kd = self._as_hand_array(default_kd, "default_kd")

    def send_hand_cmd(
        self,
        side,
        q,
        dq=None,
        tau=None,
        kp=None,
        kd=None,
        duration=None,
        hold_position=False,
    ):
        """Publish one /hand_cmd/<side>. Dropped until start_hand_control succeeds.

        `q` is 7 long, in DDS order. The two hands are independent topics, so a
        caller driving one hand must keep the other's watchdog fed or let it
        release.
        """
        if side not in HAND_SIDES:
            raise ValueError(f"side must be one of {HAND_SIDES}, got {side!r}")

        cmd = pack_hand_cmd(
            q=q,
            dq=dq if dq is not None else self._hand_zeros,
            tau=tau if tau is not None else self._hand_zeros,
            kp=self._resolve(kp, self._default_hand_kp, "kp", "set_default_hand_cmd"),
            kd=self._resolve(kd, self._default_hand_kd, "kd", "set_default_hand_cmd"),
            duration=self.hand_duration if duration is None else duration,
            hold_position=hold_position,
        )
        if len(cmd.motor_cmd) != NUM_HAND_JOINTS:
            raise ValueError(f"expected {NUM_HAND_JOINTS} motors, packed {len(cmd.motor_cmd)}")
        self.hand_cmd_publishers[side].publish(cmd)
        return cmd

    def start_hand_control(self, timeout_sec=5.0):
        """Ramp both hands to the config's ready pose, then accept /hand_cmd/*.

        Answers success=false when no hand is publishing state, and names any
        hand it skipped for that reason. Independent of start_control.
        """
        return self._call(self.start_hand_control_client, Trigger.Request(), timeout_sec)

    def stop_hand_control(self, timeout_sec=5.0):
        """Make the bridge ignore /hand_cmd/*. Blocks on the bridge."""
        return self._call(self.stop_hand_control_client, Trigger.Request(), timeout_sec)
