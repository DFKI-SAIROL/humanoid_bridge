"""Shared fakes: a config with two processes, the G1 catalog, a recording supervisor."""

from __future__ import annotations

import threading
from pathlib import Path

import pytest
from robot_console.catalog import load_catalog
from robot_console.config import parse
from robot_console.service import ConsoleService
from robot_console.session import ProbeSession

G1_CONFIG = Path(__file__).resolve().parents[2] / "robot_bridge" / "params" / "G1_config.yaml"


def make_config(**console):
    return parse(
        {
            "console": {"robot": "G1", "bridge_config": str(G1_CONFIG), "hands": True, **console},
            "processes": {
                "gear": {"argv": ["gear"], "commands_robot": True, "requires": ["armed"]},
                "camera": {"argv": ["camera"], "group": "bring-up"},
            },
        }
    )


class FakeSupervisor:
    """Records calls; `running` is the set of names that count as streaming."""

    def __init__(self):
        self.calls = []
        self.running = set()

    def start(self, name, params):
        self.calls.append(("start", name, params))
        self.running.add(name)
        return True, "started"

    def stop(self, name):
        self.calls.append(("stop", name))
        self.running.discard(name)
        return True, "stopping"

    def answer(self, name, answer):
        self.calls.append(("answer", name, answer))
        return True, "answered"

    def log(self, name, since=0):
        return {"seq": 3, "lines": ["a", "b", "c"][since:]}

    def snapshot(self):
        return [
            {"name": n, "state": "ready", "commands_robot": n == "gear"}
            for n in sorted(self.running)
        ]

    def running_commands_robot(self):
        return [n for n in sorted(self.running) if n == "gear"]


class FakeLoop:
    """Answers queued requests on a thread, the way the ROS loop does."""

    def __init__(self, service, answers=None):
        self.service = service
        self.answers = answers or {}
        self.seen = []
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.wait(0.005):
            for kind, answer in self.service.pending():
                self.seen.append(kind)
                answer.put(self.answers.get(kind, (True, f"did {kind}")))

    def close(self):
        self._stop.set()
        self._thread.join(timeout=1.0)


@pytest.fixture
def catalog():
    return load_catalog(G1_CONFIG, hands=True)


@pytest.fixture
def parts(catalog):
    """(service, supervisor, estop calls, loop) with the loop running."""
    supervisor = FakeSupervisor()
    estops = []

    def estop(mode):
        estops.append(mode)
        return True, "sent"

    service = ConsoleService(make_config(), catalog, ProbeSession(catalog), supervisor, estop)
    loop = FakeLoop(service)
    yield service, supervisor, estops, loop
    loop.close()
