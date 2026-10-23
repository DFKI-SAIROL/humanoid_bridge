"""The controller log: rows as sent, header once per file, gains only when they change."""

from __future__ import annotations

import json

import numpy as np

from g1_stack import cmd_log
from g1_stack.amo.policy import STAND


class FakeClient:
    def __init__(self):
        self.cmds = []
        self.hands = []
        self.extra = "passed through"

    def send_cmd(self, **kwargs):
        self.cmds.append(kwargs)
        return "sent"

    def send_hand_cmd(self, side, **kwargs):
        self.hands.append((side, kwargs))
        return "hand sent"


def _rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def _logged(path, episode="episode_0", header=None):
    return cmd_log.LoggedCmdClient(FakeClient(), path, episode, header or {"controller": "amo"},
                                   clock=lambda: 5.0, wall=lambda: 1700.0)


def test_no_env_leaves_the_client_alone():
    client = FakeClient()
    assert cmd_log.from_env(client, {}, environ={}) is client


def test_the_env_names_the_file_and_the_episode(tmp_path):
    path = tmp_path / "rid.task0.controller.jsonl"
    client = cmd_log.from_env(FakeClient(), {"controller": "gear-wbc"},
                              environ={"G1_CONTROLLER_LOG": str(path), "G1_EPISODE": "episode_3"})
    client.send_cmd(q_target_pos=np.zeros(29))
    client.close_log()
    header, row = _rows(path)
    assert header["controller"] == "gear-wbc" and header["format"] == 1
    assert row["episode"] == "episode_3"


def test_a_send_is_passed_on_and_logged_as_sent(tmp_path):
    path = tmp_path / "c.jsonl"
    client = _logged(path)
    q = np.arange(29, dtype=np.float64)
    assert client.send_cmd(q_target_pos=q, target_tau=np.ones(29), target_kp=np.full(29, 50.0),
                           target_kd=np.full(29, 2.0), duration=0.02) == "sent"
    q[:] = -1  # the sender reuses its buffer
    client.close_log()
    assert client._client.cmds[0]["duration"] == 0.02
    header, row = _rows(path)
    assert header == {"format": 1, "code_path": "bridge", "controller": "amo"}
    assert row["q_target"] == list(range(29))
    assert row["tau"] == [1.0] * 29
    assert row["phase"] == "policy" and row["step"] is None
    assert (row["wall_t"], row["t"], row["hold"]) == (5.0, 1700.0, False)


def test_gains_appear_only_when_they_change(tmp_path):
    path = tmp_path / "c.jsonl"
    client = _logged(path)
    for kp in (50.0, 50.0, 60.0):
        client.send_cmd(q_target_pos=np.zeros(29), target_kp=np.full(29, kp),
                        target_kd=np.full(29, 2.0))
    client.close_log()
    rows = _rows(path)[1:]
    assert [r.get("kp", [None])[0] for r in rows] == [50.0, None, 60.0]


def test_a_note_lands_on_the_next_row_only(tmp_path):
    path = tmp_path / "c.jsonl"
    client = _logged(path)
    client.note(obs={"quat": np.array([1.0, 0, 0, 0])}, command=STAND)
    client.send_cmd(q_target_pos=np.zeros(29))
    client.send_cmd(q_target_pos=np.zeros(29))
    client.close_log()
    first, second = _rows(path)[1:]
    assert first["obs"]["quat"] == [1.0, 0.0, 0.0, 0.0]
    assert first["command"]["torso_height"] == STAND.torso_height
    assert "obs" not in second


def test_hand_rows_and_passthrough(tmp_path):
    path = tmp_path / "c.jsonl"
    client = _logged(path)
    assert client.send_hand_cmd("left", q=np.ones(7), kp=[1.5] * 7) == "hand sent"
    assert client.extra == "passed through"
    client.close_log()
    row = _rows(path)[1]
    assert (row["phase"], row["side"], row["q_target"]) == ("hand", "left", [1.0] * 7)


def test_a_second_episode_appends_without_a_header(tmp_path):
    path = tmp_path / "c.jsonl"
    for episode in ("episode_0", "episode_1"):
        client = _logged(path, episode)
        client.send_cmd(q_target_pos=np.zeros(29))
        client.close_log()
        client.close_log()  # twice is harmless
    rows = _rows(path)
    assert ["format" in r for r in rows] == [True, False, False]
    assert [r["episode"] for r in rows[1:]] == ["episode_0", "episode_1"]
