"""The Unitree DDS transport the joint states arrive on.

- body joints: `unitree_hg/LowState` on `rt/lowstate`;
- Dex3 hands: `rt/dex3/{left,right}/state`, read raw; the G1 bridge's hand path reads it too.

Read through `unitree_sdk2py` rather than rclpy: the Jetson's ROS 2 workspace
has no `unitree_hg` Python message package, which is why `ros2 topic echo
/lowstate` prints nothing while `topic hz` reports 210 Hz.
"""

from __future__ import annotations

import time

LOW_STATE_TOPIC = "rt/lowstate"
DEX3_STATE_TOPICS = {"left": "rt/dex3/left/state", "right": "rt/dex3/right/state"}


class LatestFromDds:
    """The newest sample on a DDS topic, as the callable the readers take.

    `unitree_sdk2py` delivers on its own listener thread. Storing the sample is
    one attribute assignment, which is atomic under the GIL, so reading it back
    needs no lock.
    """

    def __init__(self, topic, message_type, subscriber_factory=None):
        self.topic = topic
        self._message_type = message_type
        self._factory = subscriber_factory or _channel_subscriber
        self._subscriber = None
        self._latest = None
        self.samples = 0

    def start(self) -> LatestFromDds:
        self._subscriber = self._factory(self.topic, self._message_type)
        # queueLen 0 keeps the handler on the DDS thread; we only want the last.
        self._subscriber.Init(self._on_sample, 0)
        return self

    def _on_sample(self, sample) -> None:
        self._latest = sample
        self.samples += 1

    def __call__(self):
        return self._latest

    def close(self) -> None:
        if self._subscriber is not None:
            self._subscriber.Close()
            self._subscriber = None


def _channel_subscriber(topic, message_type):
    from unitree_sdk2py.core.channel import ChannelSubscriber

    return ChannelSubscriber(topic, message_type)


_dds_initialized = False


def initialize_dds(network_interface=None, domain_id: int = 0) -> None:
    """Bring up the DDS channel factory. Once per process, before any subscriber.

    `None` auto-detects the interface, which is what running on the robot's own
    Jetson wants; pass a name only when running off-board.
    """
    global _dds_initialized
    if _dds_initialized:
        return
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize

    ChannelFactoryInitialize(domain_id, network_interface)
    _dds_initialized = True


def subscribe_low_state(**kwargs) -> LatestFromDds:
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

    return LatestFromDds(LOW_STATE_TOPIC, LowState_, **kwargs).start()


def subscribe_dex3(side: str, **kwargs) -> LatestFromDds:
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandState_

    return LatestFromDds(DEX3_STATE_TOPICS[side], HandState_, **kwargs).start()


def wait_for_sample(
    latest, deadline_s: float = 5.0, poll_s: float = 0.05, sleep=time.sleep, clock=time.monotonic
) -> bool:
    """Block until the topic has delivered once.

    Without this the first `read()` raises: DDS discovery takes longer than the
    210 Hz publish rate suggests.
    """
    started = clock()
    while clock() - started < deadline_s:
        if latest() is not None:
            return True
        sleep(poll_s)
    return latest() is not None
