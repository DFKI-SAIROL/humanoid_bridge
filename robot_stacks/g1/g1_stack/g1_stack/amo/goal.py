"""The whole-body goal for the AMO controller: one ZMQ PUB/SUB message, JSON, newest wins.

The goal source binds (the lab's psi0 node), the controller connects, as on GEAR's port 6001.
Runs on Python 3.8: the lab node imports it from its own venv.
"""

from __future__ import annotations

import json
import math
import time

import numpy as np
import zmq

from .policy import STAND_HEIGHT, Command

GOAL_PORT = 6001
TOPIC = b"amo/goal "
NUM_ARM = 14
NUM_HAND = 7


def make_goal(arm_q, hand_q=None, torso_rpy=(0.0, 0.0, 0.0), torso_height=STAND_HEIGHT,
              vx=0.0, vy=0.0, vyaw=0.0, target_yaw=0.0, stamp=None):
    """The wire dict. `arm_q`: motors 15..28; `hand_q`: left 7 then right 7, Dex3 DDS order."""
    return {
        "arm_q": [float(v) for v in arm_q],
        "hand_q": None if hand_q is None else [float(v) for v in hand_q],
        "torso_rpy": [float(v) for v in torso_rpy],
        "torso_height": float(torso_height),
        "vx": float(vx), "vy": float(vy), "vyaw": float(vyaw), "target_yaw": float(target_yaw),
        "stamp": time.monotonic() if stamp is None else float(stamp),
    }


def _floats(goal, key, n):
    values = goal.get(key)
    if not isinstance(values, list) or len(values) != n:
        raise ValueError(f"goal {key}: expected {n} values")
    out = np.array(values, dtype=np.float64)
    if not np.all(np.isfinite(out)):
        raise ValueError(f"goal {key}: non-finite value")
    return out


def _float(goal, key):
    value = float(goal[key])
    if not math.isfinite(value):
        raise ValueError(f"goal {key}: non-finite value")
    return value


def parse(goal):
    """(Command, arm_q (14,), hand_q (14,) or None, stamp); ValueError on a malformed goal."""
    try:
        roll, pitch, yaw = _floats(goal, "torso_rpy", 3)
        command = Command(
            torso_roll=roll, torso_pitch=pitch, torso_yaw=yaw,
            torso_height=_float(goal, "torso_height"),
            vx=_float(goal, "vx"), vy=_float(goal, "vy"), vyaw=_float(goal, "vyaw"),
            target_yaw=_float(goal, "target_yaw"))
        arm_q = _floats(goal, "arm_q", NUM_ARM)
        hand_q = None if goal.get("hand_q") is None else _floats(goal, "hand_q", 2 * NUM_HAND)
        return command, arm_q, hand_q, _float(goal, "stamp")
    except (KeyError, TypeError) as exc:
        raise ValueError(f"goal: {exc}") from exc


class GoalPublisher:
    """Binds; `send` never blocks."""

    def __init__(self, port=GOAL_PORT, bind_host="127.0.0.1", context=None):
        self._owns_context = context is None
        self._context = context if context is not None else zmq.Context()
        self._socket = self._context.socket(zmq.PUB)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.bind(f"tcp://{bind_host}:{port}")

    def send(self, goal):
        try:
            self._socket.send(TOPIC + json.dumps(goal).encode(), flags=zmq.NOBLOCK)
        except zmq.Again:
            pass

    def close(self):
        self._socket.close()
        if self._owns_context:
            self._context.term()


class GoalSubscriber:
    """Connects; CONFLATE keeps only the newest message. `latest()` never blocks."""

    def __init__(self, host="127.0.0.1", port=GOAL_PORT, context=None):
        self._owns_context = context is None
        self._context = context if context is not None else zmq.Context()
        self._socket = self._context.socket(zmq.SUB)
        self._socket.setsockopt(zmq.CONFLATE, 1)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.setsockopt(zmq.SUBSCRIBE, TOPIC)
        self._socket.connect(f"tcp://{host}:{port}")

    def latest(self):
        """The newest goal dict since the last call, or None."""
        try:
            raw = self._socket.recv(flags=zmq.NOBLOCK)
        except zmq.Again:
            return None
        return json.loads(raw[len(TOPIC):].decode())

    def close(self):
        self._socket.close()
        if self._owns_context:
            self._context.term()
