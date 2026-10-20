"""The ROS side: the main node on the loop thread, and an e-stop node on its own.

Every ROS import sits inside a function or method, so the rest of the package
imports without rclpy.
"""

from __future__ import annotations

import threading
import time

from .catalog import BODY, LEFT_HAND, RIGHT_HAND

# Jog publish rate. The bridge invalidates a command at duration + 0.2 s, so at
# 50 Hz with DURATION 0.04 s a dropped stream latches `hold` (cmd_timeout) about 0.24 s later.
RATE = 50.0
PERIOD = 1.0 / RATE
# Each command interpolates over two periods: a late one does not stop the joint.
DURATION = 2.0 * PERIOD

HAND_SIDES = {LEFT_HAND: "left", RIGHT_HAND: "right"}
# wireless_remote bytes 0-1 while the remote is connected; all 40 bytes are 0 when it is off.
REMOTE_HEADER = (0x55, 0x51)


def has_remote(msg_type) -> bool:
    """Whether a lowstate type carries a `wireless_remote` field."""
    return hasattr(msg_type, "wireless_remote")


def remote_header(msg) -> bool:
    """True if `msg.wireless_remote` starts with REMOTE_HEADER."""
    remote = getattr(msg, "wireless_remote", None)
    if remote is None or len(remote) < 2:
        return False
    return (int(remote[0]), int(remote[1])) == REMOTE_HEADER


class EstopLink:
    """Fires /estop_damp and /estop_hold from any thread, never behind a queue."""

    def __init__(self, on_reply=None):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from std_srvs.srv import Trigger

        self._trigger = Trigger
        self._on_reply = on_reply
        self._node = rclpy.create_node("robot_console_estop")
        self._clients = {
            mode: self._node.create_client(Trigger, f"/estop_{mode}") for mode in ("damp", "hold")
        }
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()

    def __call__(self, mode: str):
        client = self._clients[mode]
        if not client.service_is_ready():
            return False, f"/estop_{mode} is not available; use the remote (L1+R1 damp, R1+R2 hold)"
        future = client.call_async(self._trigger.Request())
        future.add_done_callback(lambda f: self._done(mode, f))
        return True, "sent"

    def _done(self, mode, future):
        if self._on_reply is None:
            return
        try:
            reply = future.result()
            self._on_reply(mode, bool(reply.success), str(reply.message))
        except Exception as exc:  # noqa: BLE001 - the page needs the reason
            self._on_reply(mode, False, str(exc))

    def close(self):
        self._executor.shutdown()
        self._node.destroy_node()


class RosLink:
    """State in, jog commands out, queued service calls run here. Loop thread only."""

    def __init__(self, node, config, catalog, service):
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from rosidl_runtime_py.utilities import get_message
        from std_msgs.msg import String
        from std_srvs.srv import Trigger

        from robot_bridge.cmd_client import RobotCmdClient

        self.node = node
        self.service = service
        self.hands = config.console.hands
        self._trigger = Trigger
        self._next_publish = 0.0
        self._num_body = len(catalog.group(BODY))
        self.client = RobotCmdClient(
            node, num_dof=self._num_body, control_frequency=RATE, interpolation_order=1.0
        )
        self._ready_client = node.create_client(Trigger, "/ready_position_control")

        lowstate_type = get_message(config.console.lowstate_type)
        service.set_remote_supported(has_remote(lowstate_type))
        node.create_subscription(lowstate_type, config.console.lowstate_topic, self._on_lowstate, 1)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        node.create_subscription(String, "/bridge_state", self._on_bridge_state, latched)

        if self.hands:
            from unitree_hg.msg import HandState

            for group, side in HAND_SIDES.items():
                node.create_subscription(
                    HandState,
                    f"/dex3/{side}/state",
                    lambda msg, g=group: self._on_hand(g, msg),
                    1,
                )

    # --- state in -------------------------------------------------------

    @staticmethod
    def _read(motor_state, count):
        motors = list(motor_state)[:count]
        return {
            "q": [float(m.q) for m in motors],
            "dq": [float(m.dq) for m in motors],
            "tau": [float(m.tau_est) for m in motors],
        }

    def _on_lowstate(self, msg):
        imu = getattr(msg, "imu_state", None)
        rpy = getattr(imu, "rpy", None) if imu is not None else None
        self.service.set_measured(BODY, self._read(msg.motor_state, self._num_body), rpy)
        if remote_header(msg):
            self.service.note_remote()

    def _on_hand(self, group, msg):
        self.service.set_measured(group, self._read(msg.motor_state, 7))

    def _on_bridge_state(self, msg):
        self.service.set_bridge_state(msg.data)

    # --- queued requests ------------------------------------------------

    def handle_requests(self):
        for kind, answer in self.service.pending():
            try:
                answer.put(self._run(kind))
            except Exception as exc:  # noqa: BLE001 - the page needs the reason
                answer.put((False, str(exc)))

    def _run(self, kind):
        client = self.client
        if kind == "arm":
            q = self.service.measured(BODY)
            if q is None:
                return False, "no /lowstate yet: is the robot (or fake_g1) publishing?"
            reply = client.start_control(default_position=q)
            return bool(reply.success), str(reply.message)
        if kind == "ready":
            reply = client._call(self._ready_client, self._trigger.Request(), 5.0)
            return bool(reply.success), str(reply.message)
        if kind == "stop":
            reply = client.stop_control()
            return bool(reply.success), str(reply.message)
        if kind == "reset":
            reply = client.estop_reset()
            return bool(reply.success), str(reply.message)
        if kind == "jog_start":
            if not self.service.latch_jog_from_measured():
                return False, "no /lowstate yet"
            message = "jog started; it streams while this tab pings"
            if self.hands:
                try:
                    reply = client.start_hand_control()
                    message += f"; hands: {reply.message}"
                except RuntimeError as exc:
                    message += f"; hands unavailable ({exc})"
            self.service.set_jog_active(True)
            return True, message
        if kind == "jog_stop":
            if not self.service.jog_active():
                return False, "no jog active"
            # Stop first, so the stream's end is a clean hold and not a cmd_timeout latch.
            reply = client.stop_control()
            self.service.set_jog_active(False)
            if self.hands:
                try:
                    client.stop_hand_control()
                except RuntimeError:
                    pass
            return True, f"jog stopped; {reply.message}"
        return False, f"unknown request {kind!r}"

    # --- jog out ----------------------------------------------------------

    def publish(self):
        """One command per group at RATE, only while a jog is active and its tab pings."""
        # spin_once returns on every /lowstate (500 Hz): pace here, not in the loop.
        now = time.monotonic()
        if now < self._next_publish:
            return
        # A fixed schedule, not now + PERIOD, so loop latency does not lower the rate.
        self._next_publish += PERIOD
        if self._next_publish < now:
            self._next_publish = now + PERIOD  # fell behind: no burst to catch up
        publish, commands = self.service.jog_commands()
        if not publish:
            return
        body = commands[BODY]
        self.client.send_cmd(
            q_target_pos=body["q"], target_kp=body["kp"], target_kd=body["kd"], duration=DURATION
        )
        if not self.hands:
            return
        for group, side in HAND_SIDES.items():
            hand = commands[group]
            try:
                self.client.send_hand_cmd(
                    side, q=hand["q"], kp=hand["kp"], kd=hand["kd"], duration=DURATION
                )
            except (ValueError, RuntimeError):
                pass
