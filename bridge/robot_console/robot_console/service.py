"""What the web server calls. Thread-safe; knows no ROS and no HTTP.

Web threads call the actions. Service calls that block (arm, reset, ...) are
queued for the ROS loop thread; the e-stop is not: it goes straight to
`estop`, so it never waits behind a slow call. Rules: docs/robot_console.md.
"""

from __future__ import annotations

import collections
import queue
import secrets
import threading
import time
from typing import Callable

from .catalog import BODY, LEFT_HAND, RIGHT_HAND
from .session import ProbeSession

ARMED = "ARMED"
EVENTS_KEPT = 30
GROUPS = (BODY, LEFT_HAND, RIGHT_HAND)
# The jog moves toward the slider at most this fast (rad/s); the page offers these.
JOG_SPEEDS = (0.2, 0.5, 1.0)
DEFAULT_JOG_SPEED = 0.5
SLEW_DT_MAX = 0.1  # a stalled loop must not turn into one big step
REQUEST_TIMEOUT = 10.0


class Refused(Exception):
    """An action the rules forbid right now; the message says why."""


class ConsoleService:
    """Control lock, jog session, robot state and the queue to the ROS loop."""

    def __init__(
        self,
        config,
        catalog,
        session: ProbeSession,
        supervisor=None,
        estop: Callable[[str], tuple[bool, str]] | None = None,
        clock: Callable[[], float] = time.monotonic,
    ):
        self._config = config
        self._catalog = catalog
        self._session = session
        self._supervisor = supervisor
        self._estop = estop
        self._clock = clock
        self._lock = threading.Lock()
        self._requests: queue.Queue = queue.Queue()
        self._events = collections.deque(maxlen=EVENTS_KEPT)

        self._clients: dict = {}  # id -> connect time
        self._controller: str | None = None
        self._controller_since: float | None = None

        self._bridge_state: str | None = None
        self._measured: dict = {}
        self._lowstate_at: float | None = None
        self._rpy = None
        self._jog_active = False
        self._jog_speed = DEFAULT_JOG_SPEED
        self._jog_q = None  # group -> the slewed q actually sent; None reseeds from the targets
        self._jog_at = 0.0

    # --- clients and the control lock -------------------------------------

    def connect(self) -> str:
        client = secrets.token_hex(6)
        with self._lock:
            self._clients[client] = self._clock()
        return client

    def disconnect(self, client: str) -> None:
        """A tab's socket closed: it loses control, and the jog deadman trips."""
        with self._lock:
            self._clients.pop(client, None)
            if self._controller == client:
                self._controller = None
                self._controller_since = None
                self._session.note_heartbeat(float("-inf"))
                self._event("warn", "the controlling tab disconnected; control is free")

    def take_control(self, client: str) -> tuple[bool, str]:
        with self._lock:
            if client not in self._clients:
                raise Refused("unknown client: reconnect the page")
            previous = self._controller
            self._controller = client
            self._controller_since = self._clock()
            self._session.note_heartbeat(float("-inf"))  # a new controller starts its own pings
            if previous and previous != client:
                self._event("warn", "control taken over by another tab")
            else:
                self._event("info", "control taken")
        return True, "you have control"

    def _require_control(self, client) -> None:
        if client is None or client != self._controller:
            raise Refused("this tab does not have control: take control first")

    def ping(self, client: str) -> None:
        """Jog deadman beat. Only the controller's pings count."""
        with self._lock:
            if client == self._controller:
                self._session.note_heartbeat(self._clock())

    # --- actions any tab may take -------------------------------------------

    def estop(self, mode: str) -> tuple[bool, str]:
        if mode not in ("damp", "hold"):
            raise ValueError(f"mode {mode!r}: expected damp or hold")
        if self._estop is None:
            return False, "no e-stop link"
        ok, message = self._estop(mode)
        with self._lock:
            self._event("bad" if not ok else "warn", f"E-STOP {mode.upper()}: {message}")
        return ok, message

    def stop(self) -> tuple[bool, str]:
        return self._request("stop")

    def process_stop(self, name: str) -> tuple[bool, str]:
        ok, message = self._sup().stop(name)
        self._log_result(f"stop {name}", ok, message)
        return ok, message

    def jog_stop(self) -> tuple[bool, str]:
        return self._request("jog_stop")

    # --- actions only the controller may take ---------------------------------

    def reset(self, client) -> tuple[bool, str]:
        with self._lock:
            self._require_control(client)
        return self._request("reset")

    def arm(self, client, confirm: bool) -> tuple[bool, str]:
        self._check_motion(client, confirm, "arm")
        return self._request("arm")

    def ready(self, client, confirm: bool) -> tuple[bool, str]:
        self._check_motion(client, confirm, "ready")
        return self._request("ready")

    def _check_motion(self, client, confirm, what) -> None:
        with self._lock:
            self._require_control(client)
            if confirm is not True:
                raise Refused(f"{what} moves the robot: confirm that it is clear")
            if self._jog_active:
                raise Refused(f"{what} refused: a jog is active, stop it first")
        streaming = self._streaming()
        if streaming:
            raise Refused(
                f"{what} refused: {', '.join(streaming)} commands the robot; stop it first"
            )

    def process_start(self, client, name: str, params=None) -> tuple[bool, str]:
        spec = self._config.process(name)  # KeyError: unknown name
        with self._lock:
            self._require_control(client)
            if spec.commands_robot and self._jog_active:
                raise Refused(f"{name} commands the robot and a jog is active: stop the jog first")
        ok, message = self._sup().start(name, params or {})
        self._log_result(f"start {name}", ok, message)
        return ok, message

    def process_answer(self, client, name: str, answer: str) -> tuple[bool, str]:
        with self._lock:
            self._require_control(client)
        ok, message = self._sup().answer(name, answer)
        self._log_result(f"{name}: answered {answer!r}", ok, message)
        return ok, message

    def jog_start(self, client) -> tuple[bool, str]:
        with self._lock:
            self._require_control(client)
            if self._bridge_state != ARMED:
                raise Refused(
                    f"jog needs the bridge ARMED, it is {self._bridge_state or 'unknown'}"
                )
        streaming = self._streaming()
        if streaming:
            raise Refused(f"jog refused: {', '.join(streaming)} commands the robot")
        return self._request("jog_start")

    def jog_select(self, client, name: str) -> None:
        with self._lock:
            self._require_control(client)
            self._session.select(name)

    def jog_target(self, client, name: str, q: float) -> float:
        with self._lock:
            self._require_control(client)
            return self._session.set_target(name, q)

    def jog_mode(self, client, limp: bool) -> None:
        with self._lock:
            self._require_control(client)
            self._session.set_limp(limp)

    def jog_speed(self, client, speed: float) -> None:
        if speed not in JOG_SPEEDS:
            raise ValueError(f"speed {speed}: expected one of {JOG_SPEEDS} rad/s")
        with self._lock:
            self._require_control(client)
            self._jog_speed = float(speed)

    # --- reads ------------------------------------------------------------------

    def catalog_json(self) -> list:
        return self._catalog.to_json()

    def armed(self) -> bool:
        with self._lock:
            return self._bridge_state == ARMED

    def process_log(self, name: str, since: int) -> dict:
        self._config.process(name)
        return self._sup().log(name, since)

    def snapshot(self, client: str | None = None) -> dict:
        processes = self._supervisor.snapshot() if self._supervisor else []
        now = self._clock()
        with self._lock:
            commanded = {g: self._session.command(g) for g in GROUPS}
            if self._jog_active and self._jog_q is not None:
                for g in GROUPS:
                    commanded[g]["q"] = self._jog_q[g]
            measured = dict(self._measured)
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
            console = self._config.console
            return {
                "t": time.time(),
                "client": client,
                "control": {
                    "held": self._controller is not None,
                    "yours": client is not None and client == self._controller,
                    "tabs": len(self._clients),
                },
                "robot": {
                    "name": console.robot,
                    "latch_valid": console.robot.upper() == "G1",
                    "hands": console.hands,
                    "camera": console.camera is not None,
                    "urdf": None if console.urdf is None else "/assets/" + console.urdf.name,
                },
                "bridge": {
                    "state": self._bridge_state,
                    "armed": self._bridge_state == ARMED,
                    "lowstate_age": None if self._lowstate_at is None else now - self._lowstate_at,
                    "rpy": self._rpy,
                },
                "jog": {
                    "active": self._jog_active,
                    "alive": self._session.is_alive(now),
                    "limp": self._session.limp(),
                    "selected": self._session.selected(),
                    "speed": self._jog_speed,
                    "speeds": list(JOG_SPEEDS),
                },
                # The jog page reads these two at the top level, as motor_probe sent them.
                "selected": self._session.selected(),
                "motors": motors,
                "processes": processes,
                "events": list(self._events),
            }

    # --- the ROS loop thread ------------------------------------------------------

    def pending(self):
        """Drain queued requests as (kind, answer_queue)."""
        while True:
            try:
                yield self._requests.get_nowait()
            except queue.Empty:
                return

    def set_bridge_state(self, state: str) -> None:
        with self._lock:
            if state != self._bridge_state:
                self._event("info", f"bridge: {state}")
            self._bridge_state = state
            if self._jog_active and state != ARMED:
                self._jog_active = False
                self._event("warn", f"jog ended: bridge is {state}")

    def set_measured(self, group: str, state: dict, rpy=None) -> None:
        with self._lock:
            self._measured[group] = state
            if group == BODY:
                self._lowstate_at = self._clock()
                if rpy is not None:
                    self._rpy = [float(v) for v in rpy]

    def measured(self, group: str = BODY):
        with self._lock:
            state = self._measured.get(group)
            return None if state is None else list(state["q"])

    def latch_jog_from_measured(self) -> bool:
        with self._lock:
            if BODY not in self._measured:
                return False
            self._session.latch({g: s["q"] for g, s in self._measured.items()})
            return True

    def jog_active(self) -> bool:
        with self._lock:
            return self._jog_active

    def set_jog_active(self, active: bool) -> None:
        with self._lock:
            self._jog_active = active
            self._jog_q = None  # the next jog starts from its latched targets
            if active:
                # The controller's next ping arms the deadman; until then nothing streams.
                self._session.note_heartbeat(float("-inf"))

    def jog_commands(self):
        """(publish?, {group: {q, kp, kd}}); q moves toward the targets at most the jog speed."""
        with self._lock:
            now = self._clock()
            publish = self._jog_active and self._session.is_alive(now)
            commands = {g: self._session.command(g) for g in GROUPS}
            if not self._jog_active:
                self._jog_q = None
                self._jog_at = now
                return False, commands
            if self._jog_q is None:
                self._jog_q = {g: list(c["q"]) for g, c in commands.items()}
            elif publish:
                # A slider click or a fast drag becomes a ramp, not a jump.
                step = self._jog_speed * min(max(now - self._jog_at, 0.0), SLEW_DT_MAX)
                for g, c in commands.items():
                    self._jog_q[g] = [
                        prev + min(max(target - prev, -step), step)
                        for prev, target in zip(self._jog_q[g], c["q"])
                    ]
            self._jog_at = now
            for g, c in commands.items():
                c["q"] = list(self._jog_q[g])
            return publish, commands

    def note(self, level: str, text: str) -> None:
        with self._lock:
            self._event(level, text)

    # --- helpers --------------------------------------------------------------------

    def _request(self, kind: str) -> tuple[bool, str]:
        """Hand `kind` to the loop thread and wait for its answer."""
        answer: queue.Queue = queue.Queue(maxsize=1)
        self._requests.put((kind, answer))
        try:
            ok, message = answer.get(timeout=REQUEST_TIMEOUT)
        except queue.Empty:
            ok, message = False, f"the control loop did not answer within {REQUEST_TIMEOUT:.0f} s"
        self._log_result(kind, ok, message)
        return ok, message

    def _streaming(self):
        return self._supervisor.running_commands_robot() if self._supervisor else []

    def _sup(self):
        if self._supervisor is None:
            raise Refused("no processes configured")
        return self._supervisor

    def _log_result(self, what, ok, message) -> None:
        with self._lock:
            self._event("info" if ok else "bad", f"{what}: {message}")

    def _event(self, level: str, text: str) -> None:
        """Lock held."""
        self._events.append({"t": time.time(), "level": level, "text": text})
