"""ConsoleService: the control lock, the safety refusals, the jog deadman."""

from __future__ import annotations

import pytest
from robot_console import service as service_mod
from robot_console.catalog import BODY
from robot_console.service import Refused


def controlled(service):
    client = service.connect()
    service.take_control(client)
    return client


def armed(service):
    service.set_bridge_state("ARMED")
    service.set_measured(BODY, {"q": [0.1] * 29, "dq": [0.0] * 29, "tau": [0.0] * 29}, [0, 0, 0])


# --- the control lock ---------------------------------------------------


def test_one_controller_and_take_over(parts):
    service, *_ = parts
    a, b = service.connect(), service.connect()
    service.take_control(a)
    assert service.snapshot(a)["control"] == {"held": True, "yours": True, "tabs": 2}
    assert service.snapshot(b)["control"]["yours"] is False
    service.take_control(b)
    assert service.snapshot(a)["control"]["yours"] is False
    assert service.snapshot(b)["control"]["yours"] is True


def test_disconnect_frees_the_lock(parts):
    service, *_ = parts
    a = controlled(service)
    service.disconnect(a)
    assert service.snapshot()["control"]["held"] is False


def test_unknown_client_cannot_take_control(parts):
    service, *_ = parts
    with pytest.raises(Refused):
        service.take_control("nobody")


@pytest.mark.parametrize(
    "action",
    [
        lambda s, c: s.reset(c),
        lambda s, c: s.arm(c, True),
        lambda s, c: s.ready(c, True),
        lambda s, c: s.process_start(c, "camera"),
        lambda s, c: s.process_answer(c, "camera", "go"),
        lambda s, c: s.jog_start(c),
        lambda s, c: s.jog_select(c, "L_ELBOW"),
        lambda s, c: s.jog_target(c, "L_ELBOW", 0.1),
        lambda s, c: s.jog_mode(c, True),
    ],
)
def test_controller_only_actions_refuse_other_tabs(parts, action):
    service, *_ = parts
    controlled(service)
    other = service.connect()
    with pytest.raises(Refused, match="control"):
        action(service, other)
    with pytest.raises(Refused, match="control"):
        action(service, None)


# --- actions any tab may take -------------------------------------------


def test_estop_needs_no_control_and_never_queues(parts):
    service, _, estops, loop = parts
    assert service.estop("damp") == (True, "sent")
    assert service.estop("hold")[0] is True
    assert estops == ["damp", "hold"]
    assert "damp" not in loop.seen and "hold" not in loop.seen
    with pytest.raises(ValueError):
        service.estop("limp")


def test_estop_without_a_link_says_so(parts, catalog):
    from robot_console.service import ConsoleService
    from robot_console.session import ProbeSession

    bare = ConsoleService(parts[0]._config, catalog, ProbeSession(catalog))
    assert bare.estop("damp") == (False, "no e-stop link")


def test_stop_and_process_stop_need_no_control(parts):
    service, supervisor, _, loop = parts
    assert service.stop() == (True, "did stop")
    assert loop.seen == ["stop"]
    supervisor.running.add("camera")
    assert service.process_stop("camera")[0] is True
    assert ("stop", "camera") in supervisor.calls


# --- the motion rules ----------------------------------------------------


def test_arm_needs_confirm(parts):
    service, *_ = parts
    client = controlled(service)
    with pytest.raises(Refused, match="confirm"):
        service.arm(client, False)
    assert service.arm(client, True) == (True, "did arm")


def test_arm_refused_while_a_robot_process_runs(parts):
    service, supervisor, *_ = parts
    client = controlled(service)
    supervisor.running.add("gear")
    with pytest.raises(Refused, match="gear"):
        service.arm(client, True)
    with pytest.raises(Refused, match="gear"):
        service.ready(client, True)
    supervisor.running.discard("gear")
    supervisor.running.add("camera")  # does not command the robot
    assert service.arm(client, True)[0] is True


def test_jog_needs_armed_and_no_stream(parts):
    service, supervisor, *_ = parts
    client = controlled(service)
    with pytest.raises(Refused, match="ARMED"):
        service.jog_start(client)
    armed(service)
    supervisor.running.add("gear")
    with pytest.raises(Refused, match="gear"):
        service.jog_start(client)
    supervisor.running.clear()
    assert service.jog_start(client) == (True, "did jog_start")


def test_while_jogging_arm_and_robot_processes_are_refused(parts):
    service, *_ = parts
    client = controlled(service)
    service.set_jog_active(True)
    with pytest.raises(Refused, match="jog"):
        service.arm(client, True)
    with pytest.raises(Refused, match="jog"):
        service.process_start(client, "gear")
    assert service.process_start(client, "camera") == (True, "started")


def test_unknown_process_is_a_key_error(parts):
    service, *_ = parts
    client = controlled(service)
    with pytest.raises(KeyError):
        service.process_start(client, "nope")


# --- the jog deadman ------------------------------------------------------


def test_only_the_controllers_ping_keeps_the_jog_alive(parts):
    service, *_ = parts
    client = controlled(service)
    other = service.connect()
    armed(service)
    assert service.latch_jog_from_measured()
    service.set_jog_active(True)
    assert service.jog_commands()[0] is False, "no ping yet"
    service.ping(other)
    assert service.jog_commands()[0] is False, "a viewer's ping does not count"
    service.ping(client)
    publish, commands = service.jog_commands()
    assert publish is True
    assert commands[BODY]["q"][0] == pytest.approx(0.1)


def test_controller_disconnect_trips_the_deadman(parts):
    service, *_ = parts
    client = controlled(service)
    armed(service)
    service.set_jog_active(True)
    service.ping(client)
    assert service.jog_commands()[0] is True
    service.disconnect(client)
    assert service.jog_commands()[0] is False


def test_leaving_armed_ends_the_jog(parts):
    service, *_ = parts
    controlled(service)
    armed(service)
    service.set_jog_active(True)
    service.set_bridge_state("LATCHED hold cmd_timeout")
    assert service.jog_active() is False
    assert any("jog ended" in e["text"] for e in service.snapshot()["events"])


# --- the queue to the loop --------------------------------------------------


def test_a_silent_loop_times_out(parts, monkeypatch):
    service, _, _, loop = parts
    loop.close()
    monkeypatch.setattr(service_mod, "REQUEST_TIMEOUT", 0.05)
    ok, message = service.stop()
    assert ok is False and "did not answer" in message


def test_snapshot_shape(parts):
    service, *_ = parts
    snap = service.snapshot()
    assert set(snap) >= {
        "t",
        "control",
        "robot",
        "bridge",
        "jog",
        "selected",
        "motors",
        "processes",
        "events",
    }
    assert snap["robot"] == {
        "name": "G1",
        "latch_valid": True,
        "hands": True,
        "camera": False,
        "urdf": None,
        "model": None,
    }
    assert len(snap["motors"]) == 43
    assert snap["bridge"]["lowstate_age"] is None


# --- the jog slew -------------------------------------------------------------


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


def slewing(parts, catalog):
    from robot_console.service import ConsoleService
    from robot_console.session import ProbeSession

    clock = Clock()
    service = ConsoleService(parts[0]._config, catalog, ProbeSession(catalog), clock=clock)
    client = controlled(service)
    armed(service)
    service.latch_jog_from_measured()
    service.set_jog_active(True)
    service.ping(client)
    return service, client, clock


def elbow(service):
    return service.jog_commands()[1][BODY]["q"][18]


def test_a_target_jump_becomes_a_ramp(parts, catalog):
    service, client, clock = slewing(parts, catalog)
    assert elbow(service) == pytest.approx(0.1), "seeded at the latched pose"
    service.jog_target(client, "L_ELBOW", 1.1)
    for _ in range(10):
        clock.now += 0.02
        service.ping(client)
        q = elbow(service)
    assert q == pytest.approx(0.1 + 0.5 * 0.2), "0.5 rad/s for 0.2 s"
    service.jog_speed(client, 1.0)
    clock.now += 0.02
    assert elbow(service) - q == pytest.approx(1.0 * 0.02)


def test_a_stalled_loop_does_not_jump(parts, catalog):
    service, client, clock = slewing(parts, catalog)
    elbow(service)
    service.jog_target(client, "L_ELBOW", 1.1)
    clock.now += 5.0  # the loop stalled
    service.ping(client)
    assert elbow(service) == pytest.approx(0.1 + 0.5 * 0.1), "one step is capped at SLEW_DT_MAX"


def test_no_progress_without_the_deadman(parts, catalog):
    service, client, clock = slewing(parts, catalog)
    elbow(service)
    service.jog_target(client, "L_ELBOW", 1.1)
    clock.now += 1.0  # no ping for a second: not alive, nothing sent
    assert elbow(service) == pytest.approx(0.1)


def test_jog_speed_choices_and_control(parts):
    service, *_ = parts
    client = controlled(service)
    service.jog_speed(client, 0.2)
    assert service.snapshot(client)["jog"]["speed"] == 0.2
    with pytest.raises(ValueError):
        service.jog_speed(client, 3.0)
    with pytest.raises(Refused):
        service.jog_speed(service.connect(), 1.0)
