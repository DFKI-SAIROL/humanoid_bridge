"""Run the console against a simulated bridge and robot, with no ROS and no hardware.

For working on the page: arm, e-stop, reset and jog all answer, the "robot" is a
first-order lag in this process. Processes from --config are real child
processes. Proves nothing about the bridge, the wiring or the motors.

    python3 -m robot_console.devserver --config examples/console.yaml
"""

from __future__ import annotations

import argparse
import threading
import time
from pathlib import Path

from . import auth
from . import server as server_mod
from .catalog import BODY, load_catalog
from .config import load
from .service import ConsoleService
from .session import ProbeSession
from .supervisor import Supervisor

EXAMPLE = Path(__file__).resolve().parents[1] / "examples" / "console.yaml"
PERIOD = 0.02
TIME_CONSTANT = 0.25  # seconds for a joint to cover most of the way to its target


class FakeBridge:
    """The bridge's /bridge_state machine, reduced to what the page shows."""

    def __init__(self):
        self.state = "DISARMED"
        self.publishing = False

    def estop(self, mode):
        if self.state.startswith("LATCHED"):
            self.state = f"LATCHED {mode} service_{mode}" + ("" if self.publishing else " silent")
        else:
            self.publishing = self.state != "DISARMED"
            self.state = f"LATCHED {mode} service_{mode}" + ("" if self.publishing else " silent")
        return True, "simulated: latched"

    def run(self, kind):
        if kind in ("arm", "ready", "jog_start"):
            if self.state.startswith("LATCHED"):
                return False, f"{self.state}; call /estop_reset first"
            if kind != "jog_start":
                self.state, self.publishing = "ARMED", True
            return True, f"simulated: {kind}"
        if kind in ("stop", "jog_stop"):
            if self.state == "ARMED":
                self.state = "STOPPED hold stop"
            return True, f"simulated: {kind}"
        if kind == "reset":
            if not self.state.startswith("LATCHED"):
                return False, "reset refused: not latched"
            parts = self.state.split()
            self.state = f"STOPPED {parts[1]} {parts[2]}" if self.publishing else "DISARMED"
            return True, self.state
        return False, f"unknown request {kind!r}"


class FakeRobot:
    """Measured q chases commanded q while a jog streams."""

    def __init__(self, service, catalog, bridge):
        self.service = service
        self.bridge = bridge
        self._q = {g: [0.0] * len(catalog.group(g)) for g in catalog.groups()}

    def step(self, dt):
        for kind, answer in self.service.pending():
            ok, message = self.bridge.run(kind)
            if ok and kind == "jog_start":
                self.service.latch_jog_from_measured()
                self.service.set_jog_active(True)
            if kind == "jog_stop":
                self.service.set_jog_active(False)
            answer.put((ok, message))
        self.service.set_bridge_state(self.bridge.state)

        publish, commands = self.service.jog_commands()
        blend = min(1.0, dt / TIME_CONSTANT)
        for group, measured in self._q.items():
            command = commands[group]
            for i, target in enumerate(command["q"]):
                goal = target if (publish and command["kp"][i]) else measured[i]
                measured[i] += (goal - measured[i]) * blend
            state = {"q": list(measured), "dq": [0.0] * len(measured), "tau": [0.0] * len(measured)}
            self.service.set_measured(group, state, [0.0, 0.0, 0.0] if group == BODY else None)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", default=str(EXAMPLE))
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--token-file", default=str(auth.DEFAULT_TOKEN_FILE))
    parser.add_argument("--vendor", default=str(server_mod.DEFAULT_VENDOR_ROOT))
    args = parser.parse_args(argv)

    config = load(args.config)
    catalog = load_catalog(config.console.bridge_config, hands=config.console.hands)
    bridge = FakeBridge()
    holder = {}
    supervisor = Supervisor(
        config.processes,
        config.console.log_dir,
        conditions=lambda: {"armed": holder["service"].armed()},
    )
    service = ConsoleService(config, catalog, ProbeSession(catalog), supervisor, bridge.estop)
    holder["service"] = service
    robot = FakeRobot(service, catalog, bridge)

    token = auth.load_or_create_token(Path(args.token_file).expanduser())
    urdf = config.console.urdf
    httpd = server_mod.serve(
        service,
        token,
        host=args.host,
        port=args.port,
        assets_root=None if urdf is None else urdf.parent,
        vendor_root=Path(args.vendor).expanduser(),
        camera_url=config.console.camera,
    )
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    print(f"SIMULATED bridge and robot. http://{args.host}:{args.port}/?token={token}", flush=True)

    last = time.monotonic()
    try:
        while True:
            time.sleep(PERIOD)
            now = time.monotonic()
            robot.step(now - last)
            last = now
    except KeyboardInterrupt:
        pass
    finally:
        supervisor.shutdown()
        httpd.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
