"""Jog one G1 motor at a time from a browser, to prove which joint it drives.

**This node commands.** It is the raw-motor counterpart to `joint_probe`, which
drives the same robot through GEAR's goal path. This one talks to the bridge
directly on `/robot_cmd` and `/hand_cmd/*`, so it tests wiring, not policy.

    motor_probe --port 8080            # then open http://127.0.0.1:8080

Pick a motor, drag its target, watch the robot. A motor that is commanded but
does not move is the fault this exists to find.

The page holds a deadman: it pings over the websocket, and the moment those
pings stop -- tab closed, laptop asleep, wifi gone -- this stops publishing and
the bridge's own command timeout releases the robot ~0.25 s later.
"""

from __future__ import annotations

import argparse
import queue
import sys
import threading
import time
from pathlib import Path

from ..motor_probe import server as server_mod
from ..motor_probe.catalog import BODY, LEFT_HAND, RIGHT_HAND, load_catalog
from ..motor_probe.session import ProbeSession

# Publish rate. The bridge invalidates a command at duration + 0.2 s, so at
# 20 Hz a dropped stream releases the robot about a quarter second later.
RATE = 20.0
PERIOD = 1.0 / RATE

NUM_BODY = 29
HAND_SIDES = {LEFT_HAND: "left", RIGHT_HAND: "right"}

# .../g1_stack/g1_stack/nodes/ -> .../g1_stack/assets, beside the package.
DEFAULT_ASSETS = Path(__file__).resolve().parents[2] / "assets"


class ProbeService:
    """What `server.py` calls. Thread-safe; the ROS work happens on the main loop.

    Session edits are cheap and taken under the lock. Service calls are not:
    they spin the node, so they are queued and run on the loop thread instead.
    """

    def __init__(self, catalog, session):
        self._catalog = catalog
        self._session = session
        self._lock = threading.Lock()
        self._requests = queue.Queue()
        self._measured = {}
        self._started = False

    # --- called from the web threads ------------------------------------

    def catalog_json(self):
        return self._catalog.to_json()

    def select(self, name):
        with self._lock:
            self._session.select(name)

    def set_target(self, name, q):
        with self._lock:
            return self._session.set_target(name, q)

    def set_mode(self, limp):
        with self._lock:
            self._session.set_limp(limp)

    def ping(self):
        with self._lock:
            self._session.note_heartbeat(time.monotonic())

    def on_stream_closed(self):
        """Forget the last beat, so the next loop pass stops commanding."""
        with self._lock:
            self._session.note_heartbeat(float("-inf"))

    def start(self):
        return self._request("start")

    def stop(self):
        return self._request("stop")

    def _request(self, kind):
        """Hand `kind` to the loop thread and wait for its answer."""
        answer = queue.Queue(maxsize=1)
        self._requests.put((kind, answer))
        try:
            return answer.get(timeout=10.0)
        except queue.Empty:
            return False, "the control loop did not answer within 10 s"

    def snapshot(self):
        with self._lock:
            selected = self._session.selected()
            limp = self._session.limp()
            commanded = {g: self._session.command(g) for g in (BODY, LEFT_HAND, RIGHT_HAND)}
            measured = dict(self._measured)
            started = self._started
        motors = []
        for entry in self._catalog.entries:
            state = measured.get(entry.group)
            motors.append(
                {
                    "name": entry.name,
                    "q_cmd": commanded[entry.group]["q"][entry.index],
                    "q_meas": state["q"][entry.index] if state else None,
                    "dq": state["dq"][entry.index] if state else None,
                    "tau": state["tau"][entry.index] if state else None,
                }
            )
        return {
            "t": time.monotonic(),
            "started": started,
            "limp": limp,
            "selected": selected,
            "motors": motors,
        }

    # --- called from the loop thread ------------------------------------

    def pending(self):
        """Drain queued start/stop requests."""
        while True:
            try:
                yield self._requests.get_nowait()
            except queue.Empty:
                return

    def publish_measured(self, measured):
        with self._lock:
            self._measured = measured

    def set_started(self, started):
        with self._lock:
            self._started = started

    def commands(self):
        """(alive, {group: {q, kp, kd}}) under one lock, so the vectors agree."""
        with self._lock:
            alive = self._session.is_alive(time.monotonic())
            return alive, {g: self._session.command(g) for g in (BODY, LEFT_HAND, RIGHT_HAND)}

    def latch_from_measured(self):
        with self._lock:
            if self._measured:
                self._session.latch({g: s["q"] for g, s in self._measured.items()})

    def measured_body_q(self):
        with self._lock:
            state = self._measured.get(BODY)
            return list(state["q"]) if state else None


class MotorProbe:
    """Owns the node, the state subscriptions and the publish loop."""

    def __init__(self, node, client, service):
        self.node = node
        self.client = client
        self.service = service
        self.started = False
        self._body = None
        self._hands = {}

        from unitree_hg.msg import HandState, LowState

        node.create_subscription(LowState, "/lowstate", self._on_body, 1)
        for group, side in HAND_SIDES.items():
            node.create_subscription(
                HandState,
                f"/dex3/{side}/state",
                lambda msg, g=group: self._on_hand(g, msg),
                1,
            )

    def _on_body(self, msg):
        # The G1 publishes 35 motor states; the first 29 are the body.
        self._body = self._read(msg.motor_state, NUM_BODY)
        self._republish()

    def _on_hand(self, group, msg):
        self._hands[group] = self._read(msg.motor_state, 7)
        self._republish()

    @staticmethod
    def _read(motor_state, count):
        motors = list(motor_state)[:count]
        return {
            "q": [float(m.q) for m in motors],
            "dq": [float(m.dq) for m in motors],
            "tau": [float(m.tau_est) for m in motors],
        }

    def _republish(self):
        measured = dict(self._hands)
        if self._body is not None:
            measured[BODY] = self._body
        self.service.publish_measured(measured)

    def handle_requests(self):
        for kind, answer in self.service.pending():
            try:
                answer.put(self.start() if kind == "start" else self.stop())
            except Exception as exc:  # noqa: BLE001 - the browser needs the reason
                answer.put((False, str(exc)))

    def start(self):
        """Start where the robot already stands, so nothing jumps on enable."""
        measured = self.service.measured_body_q()
        if measured is None:
            return False, "no /lowstate yet -- is the robot (or fake_g1) publishing?"
        self.service.latch_from_measured()
        self.client.start_control(default_position=measured)
        message = "body control started"
        try:
            answer = self.client.start_hand_control()
            message += f"; hands: {getattr(answer, 'message', 'started')}"
        except RuntimeError as exc:
            message += f"; hands unavailable ({exc})"
        self.started = True
        self.service.set_started(True)
        return True, message

    def stop(self):
        self.started = False
        self.service.set_started(False)
        self.client.stop_control()
        try:
            self.client.stop_hand_control()
        except RuntimeError:
            pass
        return True, "stopped"

    def publish(self):
        """One command per group, but only while the browser is still pinging."""
        if not self.started:
            return
        alive, commands = self.service.commands()
        if not alive:
            return
        body = commands[BODY]
        self.client.send_cmd(
            q_target_pos=body["q"],
            target_kp=body["kp"],
            target_kd=body["kd"],
            duration=PERIOD,
        )
        for group, side in HAND_SIDES.items():
            hand = commands[group]
            try:
                self.client.send_hand_cmd(
                    side,
                    q=hand["q"],
                    kp=hand["kp"],
                    kd=hand["kd"],
                    duration=PERIOD,
                )
            except (ValueError, RuntimeError):
                pass


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--host", default="127.0.0.1", help="bind address (default localhost)")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument(
        "--assets",
        default=str(DEFAULT_ASSETS),
        help="G1 description dir; run scripts/fetch_g1_description.sh to populate it",
    )
    parser.add_argument("--limp", action="store_true", help="start with idle joints slack")
    args = parser.parse_args(argv)

    import rclpy
    from rclpy.node import Node

    from robot_bridge.cmd_client import RobotCmdClient

    catalog = load_catalog()
    session = ProbeSession(catalog, limp=args.limp)
    service = ProbeService(catalog, session)

    assets = Path(args.assets)
    if not assets.is_dir():
        print(f"no assets at {assets} -- the page falls back to a list view", file=sys.stderr)
        assets = None

    # There is no auth on these routes: anything that can reach the port can move
    # the robot. Localhost is the whole access control, so say so when it is given up.
    if args.host not in ("127.0.0.1", "localhost", "::1"):
        print(
            f"WARNING: serving on {args.host}, not localhost. These routes have no "
            "authentication -- anyone who can reach this port can command the robot.",
            file=sys.stderr,
        )

    rclpy.init()
    node = Node("motor_probe")
    client = RobotCmdClient(node, num_dof=NUM_BODY, control_frequency=RATE, interpolation_order=1.0)
    probe = MotorProbe(node, client, service)

    httpd = server_mod.serve(service, host=args.host, port=args.port, assets_root=assets)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    print(f"motor probe on http://{args.host}:{args.port}")

    # One thread spins the node: RobotCmdClient's service calls spin it too, and
    # two spinners at once deadlock.
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=PERIOD)
            probe.handle_requests()
            probe.publish()
    except KeyboardInterrupt:
        pass
    finally:
        if probe.started:
            probe.stop()
        httpd.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
