"""The GEAR-through-the-bridge launcher. No ROS, no GEAR: everything injected."""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from g1_stack.nodes import gear_controller


class FakeRclpy:
    """rclpy, with the executor behaviour gear_controller waits on.

    A node reaches the global executor only when something spins it, which is
    the race `wait_for_node` closes: GEAR's simulator indexes `get_nodes()[0]`
    with no length check.
    """

    def __init__(self, events, spin_joins=True):
        self.events = events
        self.node = SimpleNamespace(destroy_node=lambda: events.append("destroy"))
        self._spin_joins = spin_joins
        self._nodes = []

    def init(self):
        self.events.append("rclpy-init")

    def create_node(self, name):
        self.events.append(("node", name))
        return self.node

    def shutdown(self):
        self.events.append("shutdown")

    def spin(self, node):
        # a real spin blocks on a daemon thread; the fake just notes the node
        self.events.append(("spin", node is self.node))
        if self._spin_joins:
            self._nodes.append(node)

    def get_global_executor(self):
        return SimpleNamespace(get_nodes=lambda: list(self._nodes))


def fake_gear_env(events):
    """Stands in for decoupled_wbc's g1_env: a module with an init_channel.

    A namespace holding a plain function, not an object with a method: the real
    thing is a module attribute, and restoring has to give back the same object.
    """

    def init_channel(config):
        events.append(("channel", config))

    return SimpleNamespace(init_channel=init_channel)


def fake_channel(events):
    """unitree_sdk2py.core.channel, carrying the real Init's flaw.

    The singleton's Init creates the DDS domain on every call and fails once
    one exists. That is what breaks GEAR's second initialisation, and what the
    guard in channel_patcher has to absorb.
    """

    class ChannelFactory:
        def Init(self, id, networkInterface=None, qos=None):
            if "domain-created" in events:
                events.append("domain-clash")
                return False
            events.append("domain-created")
            return True

    # The real template, in the shape Init consumes it.
    config = (
        '<CycloneDDS><Domain Id="any"><General><Interfaces>'
        '<NetworkInterface name="$__IF_NAME__$" priority="default" multicast="default"/>'
        "</Interfaces></General></Domain></CycloneDDS>"
    )
    return SimpleNamespace(ChannelFactory=ChannelFactory, ChannelConfigHasInterface=config)


class Harness:
    def __init__(self, loop_main=None, g1_env=None, channel=None):
        self.events = []
        self.rclpy = FakeRclpy(self.events)
        self.g1_env = g1_env if g1_env is not None else fake_gear_env(self.events)
        self.channel = channel if channel is not None else fake_channel(self.events)
        self.loop_main = loop_main or self.default_loop

    def client_cls(self, node, num_dof, control_frequency, interpolation_order):
        self.events.append(("client", num_dof, control_frequency, interpolation_order))
        return "client"

    def install(self, client, duration):
        self.events.append(("install", client, duration))

    def install_hands(self, client):
        self.events.append(("install-hands", client))

    def default_loop(self, config):
        # GEAR builds its env here, and that is what calls init_channel
        self.events.append("loop-start")
        self.g1_env.init_channel(config={"DOMAIN_ID": 0})
        self.events.append("loop-body")

    def run(self, hz=50, hands=True):
        gear_controller.run(
            SimpleNamespace(control_frequency=hz),
            self.rclpy,
            self.client_cls,
            self.install,
            self.loop_main,
            gear_controller.channel_patcher(self.g1_env, self.channel),
            out=lambda *_: None,
            install_hands=self.install_hands if hands else None,
        )

    @property
    def kinds(self):
        return [e[0] if isinstance(e, tuple) else e for e in self.events]


def test_raw_dds_creates_the_domain_before_rclpy():
    # The bug this guards: rclpy.init() first makes ChannelFactory.Init fail
    # with "create domain error", and nothing publishes rt/lowstate.
    h = Harness()
    h.run()
    assert h.kinds.index("channel") < h.kinds.index("rclpy-init")


def test_installs_the_adapter_before_the_loop_builds_the_body():
    h = Harness()
    h.run()
    assert [k for k in h.kinds if k != "spin"] == [
        "loop-start",
        "channel",
        "rclpy-init",
        "node",
        "client",
        "install",
        "install-hands",
        "loop-body",
        "destroy",
        "shutdown",
    ]
    assert ("client", gear_controller.NUM_MOTORS, 50.0, 0.0) in h.events
    assert ("install", "client", pytest.approx(0.02)) in h.events


def test_the_hand_sender_is_swapped_too():
    """`with_hands` defaults True and nothing here turns it off, so leaving
    GEAR's HandCommandSender in place means it opens rt/dex3/<side>/cmd and
    hand commands reach the motors without passing the bridge."""
    h = Harness()
    h.run()
    assert ("install-hands", "client") in h.events


def test_the_hand_sender_is_swapped_before_the_env_is_built():
    # G1ThreeFingerHand resolves HandCommandSender at construction, a few lines
    # after G1Body does, inside the same G1Env.__init__.
    h = Harness()
    h.run()
    assert h.kinds.index("install-hands") < h.kinds.index("loop-body")


def test_the_simulators_second_channel_init_does_not_clash():
    # base_sim calls ChannelFactoryInitialize again when the sim starts.
    def loop(config):
        h.g1_env.init_channel(config={"DOMAIN_ID": 0})
        factory = h.channel.ChannelFactory()
        assert factory.Init(0, "lo") is True     # g1_env's, the real one
        assert factory.Init(0, "lo") is True     # base_sim's, absorbed

    h = Harness(loop_main=loop)
    h.run()
    assert h.events.count("domain-created") == 1
    assert "domain-clash" not in h.events


def test_spins_the_node_it_created():
    h = Harness()
    h.run()
    assert ("spin", True) in h.events


def test_duration_follows_the_loop_rate():
    h = Harness()
    h.run(hz=100)
    assert ("install", "client", pytest.approx(0.01)) in h.events


def test_restores_both_patches_afterwards():
    h = Harness()
    original_init_channel = h.g1_env.init_channel
    original_factory_init = h.channel.ChannelFactory.Init
    h.run()
    assert h.g1_env.init_channel is original_init_channel
    assert h.channel.ChannelFactory.Init is original_factory_init


def test_shuts_ros_down_when_the_loop_raises_after_the_channel():
    def loop(config):
        h.g1_env.init_channel(config={"DOMAIN_ID": 0})
        raise RuntimeError("loop died")

    h = Harness(loop_main=loop)
    with pytest.raises(RuntimeError):
        h.run()
    assert h.events[-2:] == ["destroy", "shutdown"]


def test_does_not_shut_ros_down_when_it_never_came_up():
    # The loop can die before GEAR builds its env. There is no node to destroy
    # then, and calling rclpy.shutdown() without init is an error of its own.
    def loop(config):
        raise RuntimeError("bad config")

    h = Harness(loop_main=loop)
    with pytest.raises(RuntimeError):
        h.run()
    assert "rclpy-init" not in h.events
    assert "shutdown" not in h.events


def test_refuses_to_run_when_upstream_moved_init_channel():
    h = Harness(g1_env=SimpleNamespace())
    with pytest.raises(RuntimeError, match="init_channel"):
        h.run()


def test_refuses_to_run_when_upstream_moved_the_channel_factory():
    h = Harness(channel=SimpleNamespace())
    with pytest.raises(RuntimeError, match="ChannelFactory"):
        h.run()


def test_multicast_is_forced_on_so_lo_needs_no_root():
    """`multicast="default"` resolves to off on lo, and the template goes
    straight to Domain(), out of CYCLONEDDS_URI's reach. Without this a
    rehearsal needs `ip link set lo multicast on` as root on every machine."""
    h = Harness()
    seen = {}

    def loop(config):
        h.g1_env.init_channel(config={"DOMAIN_ID": 0})
        seen["config"] = h.channel.ChannelConfigHasInterface

    h.loop_main = loop
    h.run()
    assert 'multicast="true"' in seen["config"]
    assert 'multicast="default"' not in seen["config"]


def test_the_forced_config_is_put_back_afterwards():
    h = Harness()
    original = h.channel.ChannelConfigHasInterface
    h.run()
    assert h.channel.ChannelConfigHasInterface == original


def test_refuses_to_run_if_the_node_never_joins_the_executor():
    """GEAR's simulator does `get_nodes()[0]` unguarded, so returning early
    turns into an IndexError from upstream that reads as an unrelated crash."""
    h = Harness()
    h.rclpy = FakeRclpy(h.events, spin_joins=False)
    with pytest.raises(RuntimeError, match="global executor"):
        gear_controller.wait_for_node(h.rclpy, timeout=0.05, sleep=lambda _s: None)


def test_refuses_to_run_when_the_config_no_longer_says_multicast_default():
    # A silent no-op here would put the root requirement back without a word.
    channel = fake_channel([])
    channel.ChannelConfigHasInterface = "<CycloneDDS/>"
    h = Harness(channel=channel)
    with pytest.raises(RuntimeError, match="multicast"):
        h.run()
