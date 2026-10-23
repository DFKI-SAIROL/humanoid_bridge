"""The AMO goal message and its ZMQ transport. No ROS."""

from __future__ import annotations

import socket
import time

import numpy as np
import pytest

from g1_stack.amo import goal
from g1_stack.amo.policy import Command


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def test_make_goal_parses_back():
    sent = goal.make_goal(np.arange(14) * 0.1, hand_q=np.ones(14), torso_rpy=(0.1, 0.2, 0.3),
                          torso_height=0.7, vx=0.6, vy=-0.5, vyaw=0.2, target_yaw=0.4, stamp=5.0)
    command, arm_q, hand_q, stamp = goal.parse(sent)
    assert command == Command(torso_roll=0.1, torso_pitch=0.2, torso_yaw=0.3, torso_height=0.7,
                              vx=0.6, vy=-0.5, vyaw=0.2, target_yaw=0.4)
    np.testing.assert_allclose(arm_q, np.arange(14) * 0.1)
    np.testing.assert_array_equal(hand_q, np.ones(14))
    assert stamp == 5.0


def test_hands_are_optional():
    assert goal.parse(goal.make_goal(np.zeros(14)))[2] is None


@pytest.mark.parametrize("change, match", [
    ({"arm_q": [0.0] * 13}, "arm_q: expected 14"),
    ({"hand_q": [0.0] * 7}, "hand_q: expected 14"),
    ({"torso_rpy": [0.0, float("nan"), 0.0]}, "torso_rpy: non-finite"),
    ({"vx": float("inf")}, "vx: non-finite"),
    ({"vx": None}, "goal:"),
])
def test_malformed_goal_refused(change, match):
    with pytest.raises(ValueError, match=match):
        goal.parse({**goal.make_goal(np.zeros(14)), **change})


def test_missing_key_refused():
    sent = goal.make_goal(np.zeros(14))
    del sent["target_yaw"]
    with pytest.raises(ValueError, match="target_yaw"):
        goal.parse(sent)


def test_subscriber_sees_only_the_newest_goal():
    port = free_port()
    pub = goal.GoalPublisher(port=port)
    sub = goal.GoalSubscriber(port=port)
    try:
        deadline = time.monotonic() + 5.0
        while sub.latest() is None:  # the slow-joiner wait
            assert time.monotonic() < deadline, "no goal through the socket in 5 s"
            pub.send(goal.make_goal(np.zeros(14)))
            time.sleep(0.02)
        for i in range(5):
            pub.send(goal.make_goal(np.full(14, float(i))))
        time.sleep(0.1)
        newest = sub.latest()
        assert newest["arm_q"] == [4.0] * 14
        assert sub.latest() is None
    finally:
        sub.close()
        pub.close()
