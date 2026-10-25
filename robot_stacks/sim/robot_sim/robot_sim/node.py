"""MuJoCo robot in real time on its own topics (robots/<name>.py); passive 0.4 s after the last command.

Inputs: /robot_sim/gantry (Bool, hold the base), /robot_sim/remote (Bool, remote connected),
/robot_sim/buttons (String, held buttons as `R1+R2`, "" releases), /robot_sim/reset (Empty, back on
the gantry), /robot_sim/link (Float32 s: no state out, no command in; sim time runs on),
/robot_sim/reboot (Float32 s: as link, then a reset, tick from 0, and the robot's motion service).
Parameters (read-only): robot (g1, t1), mjcf, the absolute path of the loaded model.
"""

import argparse
import os
import pathlib
import sys
import threading
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Bool, Empty, Float32, String

from g1_camera import wire
from g1_camera.server import CameraServer
from robot_sim import robots
from robot_sim.camera import SimCamera
from robot_sim.guard import FIX, not_loopback
from robot_sim.physics import RobotSim

LAG_WARN_S = 0.05  # the bridge damps on state older than 0.2 s
VIEWER_PERIOD_S = 0.02
PASSIVE_AFTER_S = 0.4  # the G1's motors go passive this long after /lowcmd stops (docs/estop.md, G0)


class RobotSimNode(Node):
    def __init__(self, sim, robot, mjcf, wire_type):
        super().__init__("robot_sim")
        read_only = ParameterDescriptor(read_only=True)
        self.declare_parameter("robot", robot, read_only)
        self.declare_parameter("mjcf", str(pathlib.Path(mjcf).resolve()), read_only)
        self.sim = sim
        self.lock = threading.Lock()
        self.cmd = sim.command()
        self.cmd_time = -float("inf")  # monotonic time of the last command
        self.reset_pending = False
        self.silent_until = 0.0  # monotonic: no state out, no command in before it
        self.boot_pending = False  # the silence is a reboot: reset at its end
        self.hand_cmds = {}  # side -> HandCommand; empty: the fingers are slack
        self.hand_due = 0.0  # sim time of the next hand state
        self.wire = wire_type(self, sim, self.lock)
        self.create_subscription(Bool, "/robot_sim/gantry", self._on_gantry, 10)
        self.create_subscription(Bool, "/robot_sim/remote", self._on_remote, 10)
        self.create_subscription(String, "/robot_sim/buttons", self._on_buttons, 10)
        self.create_subscription(Empty, "/robot_sim/reset", self._on_reset, 10)
        self.create_subscription(Float32, "/robot_sim/link", self._on_link, 10)
        self.create_subscription(Float32, "/robot_sim/reboot", self._on_reboot, 10)
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)

    # --- from the wire, executor thread -----------------------------------------

    def take_cmd(self, cmd):
        if time.monotonic() < self.silent_until:
            return  # the link is down both ways
        with self.lock:
            self.cmd = cmd
            self.cmd_time = time.monotonic()

    def take_hand_cmd(self, side, cmd):
        if time.monotonic() < self.silent_until:
            return
        with self.lock:
            self.hand_cmds = {**self.hand_cmds, side: cmd}

    # --- inputs ------------------------------------------------------------------

    def _on_gantry(self, msg):
        with self.lock:
            self.sim.gantry = bool(msg.data)
        self.get_logger().info(f"gantry {'on' if msg.data else 'off'}")

    def _on_remote(self, msg):
        self.wire.set_remote(bool(msg.data))

    def _on_buttons(self, msg):
        names = [name.strip() for name in msg.data.split("+") if name.strip()]
        try:
            self.wire.press(names)
        except ValueError as exc:
            self.get_logger().error(f"/robot_sim/buttons {msg.data!r}: {exc}")

    def _on_reset(self, _msg):
        with self.lock:
            self.reset_pending = True

    def _on_link(self, msg):
        with self.lock:
            self.silent_until = time.monotonic() + max(0.0, float(msg.data))
        self.get_logger().warn(f"link down for {msg.data:.1f} s")

    def _on_reboot(self, msg):
        with self.lock:
            self.silent_until = time.monotonic() + max(0.0, float(msg.data))
            self.boot_pending = True
        self.get_logger().warn(f"rebooting: {msg.data:.1f} s")

    # --- physics thread ----------------------------------------------------------

    def _restart(self):
        """self.lock held: back on the gantry, the sim clock from 0."""
        self.sim.reset()
        self.sim.gantry = True
        self.hand_due = 0.0

    def _loop(self):
        dt = self.sim.dt
        due = time.monotonic()
        passive = self.sim.command()
        hand_period = self.wire.hand_period
        while self.running and rclpy.ok():
            now = time.monotonic()
            with self.lock:
                silent = now < self.silent_until
                if self.boot_pending and not silent:
                    self._restart()
                    self.hand_cmds = {}
                    self.cmd = self.sim.command()
                    self.boot_pending = False
                    self.wire.boot()
                if self.reset_pending:
                    self._restart()
                    self.reset_pending = False
                cmd = self.cmd if now - self.cmd_time <= PASSIVE_AFTER_S else passive
                cmd = self.wire.filter(cmd)  # the robot's own mode may take over
                self.sim.step(cmd, self.hand_cmds)
                state = self.sim.state()
                hands = {}
                if hand_period is not None and state["time"] >= self.hand_due:
                    self.hand_due = state["time"] + hand_period
                    hands = {side: self.sim.hand_state(side) for side in self.sim.hand_qadr}
                if not silent:
                    self.wire.tick(now)
            if not silent:
                self.wire.publish(state)
                self.wire.publish_hands(hands)
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
    """The g1_camera wire on the robot's camera; None, logged, when GL fails."""
    try:
        camera = SimCamera(node.sim, node.lock, size=wire.DEFAULT_SIZE, hz=args.camera_hz).start()
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(f"camera off: {exc!r}; set MUJOCO_GL=egl or osmesa")
        return None
    server = CameraServer(
        port=args.camera_port,
        source=camera,
        source_name=f"robot_sim {args.robot} {node.sim.robot.camera}",
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
    parser.add_argument("--robot", default="g1", choices=robots.NAMES)
    parser.add_argument("--mjcf", default=None, help="the model (default: the robot's; $G1_MJCF)")
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
        sys.exit(f"robot_sim refused: {refused}. On a robot NIC, the bridge would read simulated "
                 f"state; {FIX}.")  # fmt: skip
    rclpy.init(args=argv)
    args = parser.parse_args(remove_ros_args(argv)[1:])

    robot = robots.load(args.robot)
    mjcf = args.mjcf or str(robot.SPEC.default_mjcf)
    sim = RobotSim(robot.SPEC, mjcf, clearance=0.0 if args.no_gantry else args.clearance)
    sim.gantry = not args.no_gantry
    node = RobotSimNode(sim, args.robot, mjcf, robot.Wire)
    node.get_logger().info(
        f"{args.robot} {mjcf}: {sim.model.nq} qpos, dt {sim.dt * 1000:.0f} ms, "
        f"gantry {'on' if sim.gantry else 'off'} at {sim.anchor[2]:.3f} m, {node.wire.describe}"
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
