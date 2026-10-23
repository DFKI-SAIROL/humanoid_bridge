"""MuJoCo G1 on /lowstate at 500 Hz in real time; reads /lowcmd. Never publishes /lowcmd.

Inputs: /g1_sim/gantry (Bool, hold the pelvis), /g1_sim/remote (Bool, remote header on/off),
/g1_sim/keys (UInt16, remote key mask), /g1_sim/reset (Empty, back on the gantry).
Parameter: mjcf (read-only), the absolute path of the loaded model.
A model with Dex3 hands: /dex3/{left,right}/state at 100 Hz; reads /dex3/{left,right}/cmd.
"""

import argparse
import os
import pathlib
import sys
import threading
import time

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Bool, Empty, UInt16
from unitree_hg.msg import HandCmd, HandState, LowCmd, LowState, MotorState

from g1_camera import wire
from g1_camera.server import CameraServer
from g1_sim.camera import SimCamera
from g1_sim.guard import FIX, not_loopback
from g1_sim.model import (
    DEFAULT_MJCF,
    HAND_SIDES,
    NUM_HAND_JOINT,
    NUM_JOINT,
    Command,
    G1Sim,
    HandCommand,
)

MODE_MACHINE = 5  # as fake_g1; the bridge logs "G1 type:" once it is non-zero
LAG_WARN_S = 0.05  # the bridge damps on /lowstate older than 0.2 s
VIEWER_PERIOD_S = 0.02
HAND_PERIOD_S = 0.01  # 100 Hz, as the Dex3 and fake_dex3; the bridge aborts on state older than 0.2 s


class G1SimNode(Node):
    def __init__(self, sim, mjcf):
        super().__init__("g1_sim")
        self.declare_parameter(
            "mjcf", str(pathlib.Path(mjcf).resolve()), ParameterDescriptor(read_only=True)
        )
        self.sim = sim
        self.lock = threading.Lock()
        self.cmd = Command()
        self.reset_pending = False
        self.publisher = self.create_publisher(LowState, "/lowstate", 10)
        self.create_subscription(LowCmd, "/lowcmd", self._on_cmd, 10)
        self.create_subscription(Bool, "/g1_sim/gantry", self._on_gantry, 10)
        self.create_subscription(Bool, "/g1_sim/remote", self._on_remote, 10)
        self.create_subscription(UInt16, "/g1_sim/keys", self._on_keys, 10)
        self.create_subscription(Empty, "/g1_sim/reset", self._on_reset, 10)

        self.hand_cmds = {}  # side -> HandCommand; empty: the fingers are slack
        self.hand_publishers = {}
        self.hand_states = {}
        for side in HAND_SIDES if sim.hands else ():
            self.hand_publishers[side] = self.create_publisher(HandState, f"/dex3/{side}/state", 10)
            self.hand_states[side] = HandState(motor_state=[MotorState() for _ in range(NUM_HAND_JOINT)])
            self.create_subscription(
                HandCmd, f"/dex3/{side}/cmd", lambda msg, s=side: self._on_hand_cmd(s, msg), 10
            )
        self.hand_due = 0.0  # sim time of the next hand state

        self.state = LowState()
        self.state.mode_machine = MODE_MACHINE
        self.state.wireless_remote[0] = 0x55  # header bytes as the real G1 sends them
        self.state.wireless_remote[1] = 0x51
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)

    def _on_cmd(self, msg):
        cmd = Command()
        for i in range(NUM_JOINT):
            m = msg.motor_cmd[i]
            cmd.mode[i], cmd.q[i], cmd.dq[i] = m.mode, m.q, m.dq
            cmd.kp[i], cmd.kd[i], cmd.tau[i] = m.kp, m.kd, m.tau
        with self.lock:
            self.cmd = cmd

    def _on_hand_cmd(self, side, msg):
        cmd = HandCommand()
        for i in range(min(len(msg.motor_cmd), NUM_HAND_JOINT)):  # unbounded MotorCmd[] on the wire
            m = msg.motor_cmd[i]
            cmd.q[i], cmd.dq[i], cmd.kp[i], cmd.kd[i], cmd.tau[i] = m.q, m.dq, m.kp, m.kd, m.tau
        with self.lock:
            self.hand_cmds = {**self.hand_cmds, side: cmd}

    def _publish_hands(self, states):
        for side, s in states.items():
            msg = self.hand_states[side]
            for i, ms in enumerate(msg.motor_state):
                ms.q, ms.dq, ms.tau_est = float(s["q"][i]), float(s["dq"][i]), float(s["tau_est"][i])
            self.hand_publishers[side].publish(msg)

    def _on_gantry(self, msg):
        with self.lock:
            self.sim.gantry = bool(msg.data)
        self.get_logger().info(f"gantry {'on' if msg.data else 'off'}")

    def _on_remote(self, msg):
        header = (0x55, 0x51) if msg.data else (0, 0)  # remote off: the G1 sends all 40 bytes 0
        self.state.wireless_remote[0], self.state.wireless_remote[1] = header

    def _on_keys(self, msg):
        self.state.wireless_remote[2] = msg.data & 0xFF  # key mask, little-endian
        self.state.wireless_remote[3] = msg.data >> 8

    def _on_reset(self, _msg):
        with self.lock:
            self.reset_pending = True

    def _publish(self, s):
        for i in range(NUM_JOINT):
            ms = self.state.motor_state[i]
            ms.q, ms.dq = float(s["q"][i]), float(s["dq"][i])
            ms.ddq, ms.tau_est = float(s["ddq"][i]), float(s["tau_est"][i])
        imu = self.state.imu_state
        imu.quaternion = s["quaternion"].astype(np.float32).tolist()
        imu.rpy = s["rpy"].astype(np.float32).tolist()
        imu.gyroscope = s["gyroscope"].astype(np.float32).tolist()
        imu.accelerometer = s["accelerometer"].astype(np.float32).tolist()
        self.state.tick = int(s["time"] * 1000)
        self.publisher.publish(self.state)

    def _loop(self):
        dt = self.sim.dt
        due = time.monotonic()
        while self.running and rclpy.ok():
            with self.lock:
                if self.reset_pending:
                    self.sim.reset()
                    self.sim.gantry = True
                    self.reset_pending = False
                    self.hand_due = 0.0  # the sim clock starts again
                self.sim.step(self.cmd, self.hand_cmds)
                state = self.sim.state()
                hands = {}
                if self.hand_publishers and state["time"] >= self.hand_due:
                    self.hand_due = state["time"] + HAND_PERIOD_S
                    hands = {side: self.sim.hand_state(side) for side in self.hand_publishers}
            self._publish(state)
            self._publish_hands(hands)
            due += dt
            lag = time.monotonic() - due
            if lag > LAG_WARN_S:
                self.get_logger().warn(f"{lag * 1000:.0f} ms behind real time; skipping ahead")
                due = time.monotonic()
            elif lag < 0:
                time.sleep(-lag)


def start_viewer(node):
    """The MuJoCo window, synced on its own thread; None, logged, when it cannot open."""
    try:
        import mujoco.viewer

        viewer = mujoco.viewer.launch_passive(node.sim.model, node.sim.data)
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(f"viewer off: {exc!r}")
        return None

    def sync():
        while node.running and viewer.is_running():
            with node.lock:
                viewer.sync()
            time.sleep(VIEWER_PERIOD_S)

    threading.Thread(target=sync, daemon=True).start()
    node.get_logger().info(f"viewer on {os.environ.get('DISPLAY')}")
    return viewer


def start_camera(node, args):
    """The g1_camera wire on the d435 view; None, logged, when GL fails."""
    try:
        camera = SimCamera(node.sim, node.lock, size=wire.DEFAULT_SIZE, hz=args.camera_hz).start()
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(f"camera off: {exc!r}; set MUJOCO_GL=egl or osmesa")
        return None
    server = CameraServer(
        port=args.camera_port,
        source=camera,
        source_name="g1_sim d435",
        preview_port=args.preview_port,
        bind=args.camera_bind,
        capture_hz=args.camera_hz,
    ).start()
    node.get_logger().info(
        f"camera on tcp://{server.bind}:{server.port}, "
        f"preview http://{server.bind}:{server.preview_port}"
        f"/frame.jpg ({os.environ.get('MUJOCO_GL', 'default GL')})"
    )
    return server


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mjcf", default=str(DEFAULT_MJCF), help="mjlab g1.xml, or unitree g1_29dof_with_hand.xml for Dex3 ($G1_MJCF)")
    parser.add_argument("--clearance", type=float, default=0.05, help="feet above floor, m")
    parser.add_argument("--no-gantry", action="store_true", help="start on the floor, unheld")
    parser.add_argument(
        "--viewer",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="MuJoCo window (default: on when $DISPLAY is set)",
    )
    parser.add_argument("--camera-port", type=int, default=wire.DEFAULT_PORT, help="ZMQ REP")
    parser.add_argument("--preview-port", type=int, default=wire.DEFAULT_PREVIEW_PORT, help="HTTP")
    parser.add_argument("--camera-hz", type=float, default=30.0)
    parser.add_argument("--camera-bind", default="127.0.0.1", help="camera and preview address")
    parser.add_argument("--no-camera", action="store_true", help="no rendering, no camera ports")
    argv = sys.argv if argv is None else argv
    refused = not_loopback(os.environ)  # before rclpy.init: no DDS participant yet
    if refused:
        sys.exit(f"g1_sim refused: {refused}. On a robot NIC, G1_bridge would read simulated "
                 f"/lowstate; {FIX}.")  # fmt: skip
    rclpy.init(args=argv)
    args = parser.parse_args(remove_ros_args(argv)[1:])

    sim = G1Sim(args.mjcf, clearance=0.0 if args.no_gantry else args.clearance)
    sim.gantry = not args.no_gantry
    node = G1SimNode(sim, args.mjcf)
    node.get_logger().info(
        f"{args.mjcf}: {sim.model.nq} qpos, dt {sim.dt * 1000:.0f} ms, "
        f"gantry {'on' if sim.gantry else 'off'} at {sim.anchor[2]:.3f} m, "
        f"{'Dex3 on /dex3/*' if sim.hands else 'no Dex3'}"
    )
    # Both before the physics thread: GL setup stalls it.
    server = None if args.no_camera else start_camera(node, args)
    show = bool(os.environ.get("DISPLAY")) if args.viewer is None else args.viewer
    viewer = start_viewer(node) if show else None
    node.thread.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.thread.join(timeout=1.0)
        if server is not None:
            server.stop()
        if viewer is not None:
            viewer.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
