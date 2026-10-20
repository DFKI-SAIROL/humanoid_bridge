"""ros.py's pure parts: the remote header on /lowstate. No rclpy."""

from __future__ import annotations

import types

import numpy as np
from robot_console.ros import RosLink, has_remote, remote_header

CONNECTED = [0x55, 0x51] + [0] * 38


class LowState:
    """The fields RosLink reads from unitree_hg/msg/LowState."""

    def __init__(self, wireless_remote):
        self.wireless_remote = np.array(wireless_remote, dtype=np.uint8)
        self.motor_state = [types.SimpleNamespace(q=0.0, dq=0.0, tau_est=0.0)] * 35
        self.imu_state = types.SimpleNamespace(rpy=[0.0, 0.0, 0.0])


def test_remote_header():
    assert remote_header(LowState(CONNECTED)) is True
    assert remote_header(LowState([0] * 40)) is False, "remote off: all zeros"
    assert remote_header(LowState([0x51, 0x55] + [0] * 38)) is False
    assert remote_header(types.SimpleNamespace(wireless_remote=[0x55])) is False
    assert remote_header(types.SimpleNamespace()) is False, "a type without the field"


def test_has_remote_reads_the_type():
    # rclpy message classes expose each field as a class property.
    assert has_remote(type("HgLowState", (), {"wireless_remote": property(lambda s: [])})) is True
    assert has_remote(type("OtherLowState", (), {})) is False


def test_lowstate_with_the_header_reaches_the_snapshot(parts):
    service, *_ = parts
    service.set_remote_supported(True)
    link = types.SimpleNamespace(service=service, _num_body=29, _read=RosLink._read)
    RosLink._on_lowstate(link, LowState([0] * 40))
    assert service.snapshot()["bridge"]["remote"] is False
    RosLink._on_lowstate(link, LowState(CONNECTED))
    assert service.snapshot()["bridge"]["remote"] is True
