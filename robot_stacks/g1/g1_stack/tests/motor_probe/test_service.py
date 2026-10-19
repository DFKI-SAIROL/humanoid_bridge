"""ProbeService: the thread-safe glue between the web handler and the ROS loop.

Imports `g1_stack.nodes.motor_probe`, which is ROS-free at module level -- rclpy
and unitree_hg are imported inside the functions that need them.
"""

import threading
import time

import pytest

from g1_stack.motor_probe.catalog import BODY, LEFT_HAND, RIGHT_HAND, load_catalog
from g1_stack.motor_probe.session import ProbeSession
from g1_stack.nodes.motor_probe import ProbeService


@pytest.fixture
def service():
    catalog = load_catalog()
    return ProbeService(catalog, ProbeSession(catalog))


def test_catalog_json_carries_all_43_motors(service):
    assert len(service.catalog_json()) == 43


def test_set_target_clamps_to_the_motors_travel(service):
    # L_ELBOW is [-1.0472, 2.0944].
    assert service.set_target("L_ELBOW", 99.0) == pytest.approx(2.0944)
    assert service.set_target("L_ELBOW", -99.0) == pytest.approx(-1.0472)


def test_snapshot_shape(service):
    service.select("L_ELBOW")
    snapshot = service.snapshot()
    assert len(snapshot["motors"]) == 43
    assert snapshot["selected"] == "L_ELBOW"
    assert snapshot["started"] is False
    # No state has arrived, so measurements read as None rather than a stale zero.
    assert all(motor["q_meas"] is None for motor in snapshot["motors"])


def test_snapshot_reports_measured_state_once_it_arrives(service):
    service.publish_measured({BODY: {"q": [0.5] * 29, "dq": [0.0] * 29, "tau": [1.0] * 29}})
    motors = {m["name"]: m for m in service.snapshot()["motors"]}
    assert motors["L_ELBOW"]["q_meas"] == pytest.approx(0.5)
    assert motors["L_ELBOW"]["tau"] == pytest.approx(1.0)
    # The hands still have no state of their own.
    assert motors["L_HAND_THUMB_0"]["q_meas"] is None


def test_deadman_starts_dead_and_dies_when_the_stream_closes(service):
    assert service.commands()[0] is False, "must not command before the page has ever pinged"
    service.ping()
    assert service.commands()[0] is True
    service.on_stream_closed()
    assert service.commands()[0] is False, "a closed stream must stop commanding at once"


def test_commands_cover_every_group_at_the_right_width(service):
    service.ping()
    _, commands = service.commands()
    assert {group: len(cmd["q"]) for group, cmd in commands.items()} == {
        BODY: 29,
        LEFT_HAND: 7,
        RIGHT_HAND: 7,
    }


def test_limp_powers_only_the_selected_motor(service):
    service.select("L_ELBOW")
    service.set_mode(True)
    service.ping()
    _, commands = service.commands()
    powered = [i for i, kp in enumerate(commands[BODY]["kp"]) if kp]
    assert powered == [18], "only L_ELBOW (idx 18) stays powered"
    # Both hands go slack too, but are still commanded so their watchdog stays fed.
    assert not any(commands[LEFT_HAND]["kp"])
    assert len(commands[RIGHT_HAND]["q"]) == 7


def test_hold_mode_powers_everything(service):
    service.set_mode(False)
    service.ping()
    _, commands = service.commands()
    assert all(kp > 0 for kp in commands[BODY]["kp"])


def test_latch_seeds_desired_from_measured(service):
    service.publish_measured({BODY: {"q": [0.25] * 29, "dq": [0.0] * 29, "tau": [0.0] * 29}})
    service.latch_from_measured()
    service.ping()
    _, commands = service.commands()
    # Held where the robot actually was, not at zero.
    assert commands[BODY]["q"][18] == pytest.approx(0.25)


def test_measured_body_q_is_none_until_state_arrives(service):
    assert service.measured_body_q() is None
    service.publish_measured({BODY: {"q": [0.1] * 29, "dq": [0.0] * 29, "tau": [0.0] * 29}})
    assert len(service.measured_body_q()) == 29


def test_start_is_queued_and_answered_by_the_loop_thread(service):
    """start() blocks on the web thread until the loop thread answers it."""
    answers = []

    def loop():
        # Stand in for the ROS loop: drain the queue and answer.
        for kind, answer in service.pending():
            answers.append(kind)
            answer.put((True, "started"))

    caller = threading.Thread(target=lambda: answers.append(service.start()))
    caller.start()
    for _ in range(200):
        loop()
        if not caller.is_alive():
            break
        time.sleep(0.01)
    caller.join(timeout=2.0)
    assert "start" in answers
    assert (True, "started") in answers
