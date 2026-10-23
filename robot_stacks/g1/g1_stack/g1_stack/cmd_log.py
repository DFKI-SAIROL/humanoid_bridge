"""Every /robot_cmd and /hand_cmd a controller sends, as JSONL rows: the lab's controller log.

$G1_CONTROLLER_LOG names the file, appended; the header goes only into an empty one, so one
file holds every episode of a task. $G1_EPISODE is the episode the rows carry. A writer thread
turns the rows into JSON, off the control tick. Schema: humanoid-locoman-vla
docs/run-manifest.md, "Controller log".
"""

from __future__ import annotations

import atexit
import dataclasses
import json
import os
import queue
import threading
import time
from pathlib import Path

import numpy as np

FORMAT = 1
LOG_ENV = "G1_CONTROLLER_LOG"
EPISODE_ENV = "G1_EPISODE"
_STOP = object()


def from_env(client, header, environ=None):
    """`client` wrapped in a LoggedCmdClient when $G1_CONTROLLER_LOG is set; else `client`."""
    environ = os.environ if environ is None else environ
    path = environ.get(LOG_ENV)
    if not path:
        return client
    return LoggedCmdClient(client, path, environ.get(EPISODE_ENV), header)


class LoggedCmdClient:
    """`client` with each send also logged; everything else passes through."""

    def __init__(self, client, path, episode, header, clock=time.monotonic, wall=time.time):
        self._client = client
        self._episode = episode
        self._clock = clock
        self._wall = wall
        self._note = {}
        self._gains = None  # the kp, kd last written; a row carries them only when they change
        self.errors = 0
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        new = not path.exists() or path.stat().st_size == 0
        self._file = open(path, "a", buffering=1)
        self._queue = queue.SimpleQueue()
        if new:
            self._queue.put({"format": FORMAT, "code_path": "bridge", **header})
        self._thread = threading.Thread(target=self._write, daemon=True)
        self._thread.start()
        atexit.register(self.close_log)

    def __getattr__(self, name):
        return getattr(self._client, name)

    def note(self, **fields):
        """Fields (obs, command) for the next /robot_cmd row."""
        self._note = fields

    def send_cmd(self, q_target_pos=None, q_target_vel=None, target_tau=None, target_kp=None,
                 target_kd=None, duration=None, hold_position=False):
        out = self._client.send_cmd(
            q_target_pos=q_target_pos, q_target_vel=q_target_vel, target_tau=target_tau,
            target_kp=target_kp, target_kd=target_kd, duration=duration,
            hold_position=hold_position)
        row = {"episode": self._episode, "phase": "policy", "step": None,
               "wall_t": self._clock(), "t": self._wall(), "q_target": _copy(q_target_pos),
               "tau": _copy(target_tau), "kp": _copy(target_kp), "kd": _copy(target_kd),
               "hold": hold_position, **self._note}
        self._note = {}
        self._queue.put(row)
        return out

    def send_hand_cmd(self, side, q, dq=None, tau=None, kp=None, kd=None, duration=None,
                      hold_position=False):
        out = self._client.send_hand_cmd(side, q=q, dq=dq, tau=tau, kp=kp, kd=kd,
                                         duration=duration, hold_position=hold_position)
        self._queue.put({"episode": self._episode, "phase": "hand", "side": side,
                         "wall_t": self._clock(), "t": self._wall(), "q_target": _copy(q),
                         "hold": hold_position})
        return out

    def close_log(self):
        """Write what is queued, then close. Safe to call twice."""
        if self._file.closed:
            return
        self._queue.put(_STOP)
        self._thread.join(timeout=5.0)
        self._file.close()

    def _write(self):
        while True:
            row = self._queue.get()
            if row is _STOP:
                return
            if row.get("phase") == "policy":
                gains = (row.pop("kp"), row.pop("kd"))
                if not _same(gains, self._gains):
                    row["kp"], row["kd"] = gains
                    self._gains = gains
            try:
                self._file.write(json.dumps(row, default=_plain) + "\n")
            except Exception:  # noqa: BLE001 - a lost row never stops the controller
                self.errors += 1


def _copy(value):
    """The array as sent: the sender may reuse its buffer before the writer runs."""
    return None if value is None else np.array(value, dtype=np.float64)


def _same(a, b):
    if b is None:
        return False
    return all((x is None and y is None) or (x is not None and y is not None and np.array_equal(x, y))
               for x, y in zip(a, b))


def _plain(value):
    if isinstance(value, np.ndarray):
        return [round(float(v), 6) for v in value.ravel()]
    if isinstance(value, np.generic):
        return value.item()
    if dataclasses.is_dataclass(value):
        return dataclasses.asdict(value)
    raise TypeError(f"not JSON: {type(value).__name__}")
