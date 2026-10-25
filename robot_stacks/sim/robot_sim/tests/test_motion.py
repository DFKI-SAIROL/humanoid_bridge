"""The fake motion switcher: no ROS, no mujoco."""

import json

from robot_sim.motion import (
    BAD_PARAMETER,
    BOOT_MODE,
    CHECK_MODE,
    NOT_IMPLEMENTED,
    OK,
    RELEASE_MODE,
    SELECT_MODE,
    MotionSwitcher,
)


def check(switcher):
    code, data = switcher.call(CHECK_MODE, "{}")
    assert code == OK
    return json.loads(data)["name"]


def test_starts_released_by_default():
    assert check(MotionSwitcher()) == ""


def test_release_empties_the_mode():
    switcher = MotionSwitcher(BOOT_MODE)
    assert check(switcher) == BOOT_MODE
    assert switcher.call(RELEASE_MODE, "{}") == (OK, "")
    assert check(switcher) == ""
    assert switcher.call(RELEASE_MODE, "{}") == (OK, "")  # released twice: still fine


def test_select_takes_the_name():
    switcher = MotionSwitcher()
    assert switcher.call(SELECT_MODE, json.dumps({"name": "ai"})) == (OK, "")
    assert check(switcher) == "ai"


def test_select_without_a_name_is_refused():
    switcher = MotionSwitcher()
    assert switcher.call(SELECT_MODE, "{}")[0] == BAD_PARAMETER
    assert switcher.call(SELECT_MODE, "not json")[0] == BAD_PARAMETER
    assert check(switcher) == ""


def test_other_apis_are_not_implemented():
    assert MotionSwitcher().call(1004, "{}")[0] == NOT_IMPLEMENTED
