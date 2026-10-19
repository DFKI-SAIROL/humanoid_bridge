"""Jog-panel state: desired q, limp gains, clamping and the heartbeat window."""

from __future__ import annotations

import json
from dataclasses import dataclass

import pytest

from g1_stack.motor_probe.session import BODY, LEFT_HAND, RIGHT_HAND, ProbeSession

NUM_BODY = 29
NUM_HAND = 7


@dataclass(frozen=True)
class FakeEntry:
    name: str
    group: str
    index: int
    q_min: float
    q_max: float
    dq_limit: float
    tau_limit: float
    kp: float
    kd: float
    urdf_joint: str


class FakeCatalog:
    """Stand-in for the real catalog, so these tests never read the YAML."""

    def __init__(self, entries):
        self.entries = tuple(entries)

    def by_name(self, name):
        for entry in self.entries:
            if entry.name == name:
                return entry
        raise KeyError(name)

    def group(self, group):
        return tuple(sorted((e for e in self.entries if e.group == group), key=lambda e: e.index))


def _entry(prefix, group, index):
    """One motor with limits and gains unique to it, so a mixup shows up."""
    tag = f"{prefix}{index:02d}"
    return FakeEntry(
        name=tag,
        group=group,
        index=index,
        q_min=-1.0 - index,
        q_max=2.0 + index,
        dq_limit=10.0,
        tau_limit=20.0,
        kp=100.0 + index,
        kd=1.0 + index,
        urdf_joint=f"{tag}_joint",
    )


@pytest.fixture
def catalog():
    entries = [_entry("B", BODY, i) for i in range(NUM_BODY)]
    entries += [_entry("L", LEFT_HAND, i) for i in range(NUM_HAND)]
    entries += [_entry("R", RIGHT_HAND, i) for i in range(NUM_HAND)]
    return FakeCatalog(entries)


@pytest.fixture
def session(catalog):
    return ProbeSession(catalog)


def test_the_fake_catalog_matches_the_real_one_in_size(catalog):
    assert len(catalog.entries) == NUM_BODY + 2 * NUM_HAND == 43


def test_hold_mode_gives_every_motor_its_config_gains(session, catalog):
    for group in (BODY, LEFT_HAND, RIGHT_HAND):
        command = session.command(group)
        entries = catalog.group(group)
        assert command["kp"] == [entry.kp for entry in entries]
        assert command["kd"] == [entry.kd for entry in entries]


def test_limp_mode_powers_only_the_selected_motor(session, catalog):
    session.select("B07")
    session.set_limp(True)

    body = session.command(BODY)
    selected = catalog.by_name("B07")
    assert body["kp"][7] == selected.kp
    assert body["kd"][7] == selected.kd
    assert [kp for i, kp in enumerate(body["kp"]) if i != 7] == [0.0] * (NUM_BODY - 1)
    assert [kd for i, kd in enumerate(body["kd"]) if i != 7] == [0.0] * (NUM_BODY - 1)
    for hand in (LEFT_HAND, RIGHT_HAND):
        command = session.command(hand)
        assert command["kp"] == [0.0] * NUM_HAND
        assert command["kd"] == [0.0] * NUM_HAND


def test_limp_can_be_set_at_construction(catalog):
    session = ProbeSession(catalog, limp=True)
    session.select("B02")

    assert session.command(BODY)["kp"][2] == catalog.by_name("B02").kp
    assert session.command(BODY)["kp"][3] == 0.0


def test_limp_mode_with_nothing_selected_zeroes_everything(session):
    session.set_limp(True)

    assert session.selected() is None
    for group in (BODY, LEFT_HAND, RIGHT_HAND):
        command = session.command(group)
        assert command["kp"] == [0.0] * len(command["kp"])
        assert command["kd"] == [0.0] * len(command["kd"])


def test_limp_mode_still_emits_the_stored_targets(session):
    session.set_target("B03", 1.5)
    session.select("B10")
    session.set_limp(True)

    assert session.command(BODY)["q"][3] == pytest.approx(1.5)


def test_leaving_limp_mode_restores_the_config_gains(session, catalog):
    session.select("B07")
    session.set_limp(True)
    session.set_limp(False)

    assert session.command(BODY)["kp"] == [entry.kp for entry in catalog.group(BODY)]


def test_set_target_clamps_at_both_ends_and_returns_the_clamped_value(session, catalog):
    entry = catalog.by_name("B05")

    assert session.set_target("B05", 99.0) == pytest.approx(entry.q_max)
    assert session.command(BODY)["q"][5] == pytest.approx(entry.q_max)
    assert session.set_target("B05", -99.0) == pytest.approx(entry.q_min)
    assert session.command(BODY)["q"][5] == pytest.approx(entry.q_min)


def test_set_target_passes_a_value_inside_the_travel_through(session):
    assert session.set_target("L03", 0.25) == pytest.approx(0.25)
    assert session.command(LEFT_HAND)["q"][3] == pytest.approx(0.25)


def test_set_target_moves_exactly_one_entry(session):
    session.set_target("B12", 0.4)

    q = session.command(BODY)["q"]
    assert q[12] == pytest.approx(0.4)
    assert [value for i, value in enumerate(q) if i != 12] == [0.0] * (NUM_BODY - 1)


def test_set_target_needs_no_selection(session):
    session.set_target("R02", 0.3)

    assert session.selected() is None
    assert session.command(RIGHT_HAND)["q"][2] == pytest.approx(0.3)


def test_set_target_rejects_an_unknown_motor(session):
    with pytest.raises(ValueError):
        session.set_target("NOPE", 0.0)


def test_latch_seeds_desired_from_measured(session):
    measured = {
        BODY: [0.01 * i for i in range(NUM_BODY)],
        LEFT_HAND: [0.1 * i for i in range(NUM_HAND)],
        RIGHT_HAND: [-0.1 * i for i in range(NUM_HAND)],
    }
    session.latch(measured)

    assert session.command(BODY)["q"] == pytest.approx(measured[BODY])
    assert session.command(LEFT_HAND)["q"] == pytest.approx(measured[LEFT_HAND])
    assert session.command(RIGHT_HAND)["q"] == pytest.approx(measured[RIGHT_HAND])


def test_latch_leaves_an_omitted_group_at_zeros(session):
    session.latch({BODY: [0.5] * NUM_BODY})

    assert session.command(BODY)["q"] == [0.5] * NUM_BODY
    assert session.command(LEFT_HAND)["q"] == [0.0] * NUM_HAND
    assert session.command(RIGHT_HAND)["q"] == [0.0] * NUM_HAND


def test_latch_clamps_a_measurement_outside_the_travel(session, catalog):
    session.latch({LEFT_HAND: [99.0] * NUM_HAND})

    assert session.command(LEFT_HAND)["q"] == [e.q_max for e in catalog.group(LEFT_HAND)]


def test_latch_rejects_a_wrong_length_group(session):
    with pytest.raises(ValueError):
        session.latch({BODY: [0.0] * (NUM_BODY - 1)})


def test_both_hands_are_commanded_while_a_body_joint_is_selected(session):
    session.select("B00")

    for hand in (LEFT_HAND, RIGHT_HAND):
        command = session.command(hand)
        assert len(command["q"]) == len(command["kp"]) == len(command["kd"]) == NUM_HAND


def test_body_command_is_twenty_nine_long(session):
    command = session.command(BODY)

    assert len(command["q"]) == len(command["kp"]) == len(command["kd"]) == NUM_BODY


def test_command_rejects_an_unknown_group(session):
    with pytest.raises(ValueError):
        session.command("third_hand")


def test_select_rejects_an_unknown_motor(session):
    with pytest.raises(ValueError):
        session.select("L_KNEE")


def test_select_reports_the_selected_motor(session):
    session.select("L05")

    assert session.selected() == "L05"


def test_heartbeat_is_dead_before_any_beat(session):
    assert session.is_alive(0.0) is False
    assert session.is_alive(1000.0) is False


def test_heartbeat_is_alive_inside_the_window_and_dead_after_it(catalog):
    session = ProbeSession(catalog, heartbeat_timeout=0.5)
    session.note_heartbeat(10.0)

    assert session.is_alive(10.0) is True
    assert session.is_alive(10.4) is True
    assert session.is_alive(10.6) is False


def test_a_fresh_beat_revives_the_session(session):
    session.note_heartbeat(10.0)
    assert session.is_alive(20.0) is False

    session.note_heartbeat(20.0)
    assert session.is_alive(20.1) is True


def test_command_is_json_serializable(session):
    session.latch({BODY: [0.1] * NUM_BODY})
    session.select("B01")
    session.set_limp(True)

    for group in (BODY, LEFT_HAND, RIGHT_HAND):
        payload = json.loads(json.dumps(session.command(group)))
        assert sorted(payload) == ["kd", "kp", "q"]
        assert all(isinstance(value, float) for value in payload["q"])
