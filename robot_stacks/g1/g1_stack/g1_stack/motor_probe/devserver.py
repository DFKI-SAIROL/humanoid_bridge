"""Run the panel against a simulated robot, with no ROS and no hardware.

For working on the page itself: the whole UI is live -- click a joint, drag a
target, watch the model follow -- but the "robot" is a first-order lag in this
process. It proves nothing about the bridge, the wiring or the real motors.

    python -m g1_stack.motor_probe.devserver          # then open the URL

The real node is `g1_stack.nodes.motor_probe`.
"""

from __future__ import annotations

import argparse
import threading
import time
import webbrowser
from pathlib import Path

from ..nodes.motor_probe import ProbeService
from . import server as server_mod
from .catalog import BODY, LEFT_HAND, RIGHT_HAND, load_catalog
from .session import ProbeSession

# .../g1_stack/g1_stack/motor_probe/ -> .../g1_stack/assets, beside the package.
ASSETS = Path(__file__).resolve().parents[2] / "assets"

PERIOD = 0.02
# Seconds for a joint to cover most of the distance to its target.
TIME_CONSTANT = 0.25

GROUP_SIZES = ((BODY, 29), (LEFT_HAND, 7), (RIGHT_HAND, 7))


class FakeRobot:
    """Measured q chases commanded q. Slack joints (kp=0) drift back to zero."""

    def __init__(self, service):
        self.service = service
        self.started = False
        self._q = {group: [0.0] * size for group, size in GROUP_SIZES}

    def step(self, dt):
        for kind, answer in self.service.pending():
            self.started = kind == "start"
            answer.put((True, "simulated robot: control started" if self.started else "stopped"))

        alive, commands = self.service.commands()
        blend = min(1.0, dt / TIME_CONSTANT)
        for group, measured in self._q.items():
            command = commands[group]
            for i, target in enumerate(command["q"]):
                # Only a powered joint tracks; a limp one sags toward zero.
                goal = target if (self.started and alive and command["kp"][i]) else 0.0
                measured[i] += (goal - measured[i]) * blend
        self.service.publish_measured(
            {
                group: {"q": list(q), "dq": [0.0] * len(q), "tau": [0.0] * len(q)}
                for group, q in self._q.items()
            }
        )
        self.service.set_started(self.started)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--open", action="store_true", help="open a browser at the URL")
    args = parser.parse_args(argv)

    catalog = load_catalog()
    service = ProbeService(catalog, ProbeSession(catalog))
    robot = FakeRobot(service)

    assets = ASSETS if ASSETS.is_dir() else None
    if assets is None:
        print("no assets: the page will fall back to its list view.")
        print("run scripts/fetch_g1_description.sh for the 3D model.")

    httpd = server_mod.serve(service, host=args.host, port=args.port, assets_root=assets)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    url = f"http://{args.host}:{args.port}"
    print(f"SIMULATED robot -- no ROS, no hardware. {url}")
    if args.open:
        webbrowser.open(url)

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
        httpd.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
