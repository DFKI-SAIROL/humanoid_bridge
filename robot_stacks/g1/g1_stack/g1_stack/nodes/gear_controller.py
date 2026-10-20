"""The GEAR WBC control loop, launched through the DFKI bridge.

Same flags as upstream's `run_g1_control_loop`; the only change is the command
sender: GEAR -> robot_bridge.adapters.gear_wbc -> /robot_cmd -> G1_bridge
-> /lowcmd. GEAR opens no rt/lowcmd publisher, so the bridge can start.

Arming is the operator's step, not this process's: the bridge drops every
/robot_cmd until `start_control` is called. `tools/g1-bringup.sh` does the
whole ordering.

Runs in the controller's own venv ($SIMPLE_VENV) with the ROS workspace
sourced, because it imports both GEAR and rclpy:

    source scripts/setup_unitree.sh
    PYTHONPATH=.:$PYTHONPATH \\
        "$SIMPLE_VENV"/bin/python -m g1_stack.nodes.gear_controller \\
        --interface real --enable-waist --messaging-backend zmq --zmq-host 127.0.0.1
"""

from __future__ import annotations

import sys
import threading
import time

NODE_NAME = "gear_wbc_bridge"
NUM_MOTORS = 29


def run(config, rclpy, client_cls, install, loop_main, patch_channel, out=print,
        install_hands=None):
    """Start ROS inside GEAR's channel init, then hand `config` to upstream.

    The order is the whole point. GEAR's raw CycloneDDS has to create the DDS
    domain before rclpy does. `ChannelFactory.Init` calls `Domain(id, config)`
    (`unitree_sdk2py/core/channel.py`), which cannot create a domain that
    rmw_cyclonedds already made: it prints "[ChannelFactory] create domain
    error", returns False, and nothing ever publishes `rt/lowstate`, so the
    bridge waits for a publisher forever.

    GEAR calls `init_channel` immediately before building `G1Body`, and `G1Body`
    resolves `BodyCommandSender` at construction, so this one hook is both late
    enough for the channel and early enough for the sender. `G1ThreeFingerHand`
    resolves `HandCommandSender` at construction too, a few lines later in the
    same `G1Env.__init__`, so the hands install in the same window.

    Both senders are swapped unconditionally, whatever `with_hands` says. It
    defaults True and no config here turns it off, so leaving the hand sender
    alone means GEAR opens `rt/dex3/<side>/cmd` itself and hand commands reach
    the motors without ever passing the bridge -- the one thing this whole path
    exists to prevent.
    """
    hz = float(config.control_frequency)
    started = {}

    def on_init_channel(real_init, gear_config):
        real_init(config=gear_config)   # raw DDS first: it creates the domain
        rclpy.init()                    # rmw_cyclonedds joins the existing one
        started["ros"] = True           # recorded before anything else can raise
        node = rclpy.create_node(NODE_NAME)
        # Spin on a thread: a node joins the global executor only when spun, and
        # GEAR's MuJoCo simulator takes the executor's first node for its rate
        # (base_sim.py, BaseSimulator.__init__). Publishing needs no spin; the
        # client's service calls do, and this loop makes none.
        threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
        started["node"] = node
        # Then wait for it to actually be there. base_sim takes
        # `get_global_executor().get_nodes()[0]` with no length check, and the
        # node only appears once the daemon thread reaches `executor.add_node`.
        # Hooking here leaves far less work between the two than starting the
        # thread at the top of run() did, so the race is real: losing it is an
        # IndexError from base_sim that reads as an unrelated upstream crash.
        wait_for_node(rclpy)
        # GEAR's own timing: a step every tick, no extra delay.
        client = client_cls(node, num_dof=NUM_MOTORS, control_frequency=hz, interpolation_order=0.0)
        install(client, duration=1.0 / hz)
        out(
            f"GEAR -> /robot_cmd at {hz:g} Hz, duration {1.0 / hz:.3f}s. The bridge must be "
            "running and armed with start_control."
        )
        if install_hands is not None:
            # Its own rate: GEAR's hand loop runs at 100 Hz, not the body's.
            install_hands(client)
            out("GEAR -> /hand_cmd/{left,right}. Armed separately, with start_hand_control.")

    restore = patch_channel(on_init_channel)
    try:
        loop_main(config)
    finally:
        restore()
        node = started.get("node")
        if node is not None:
            node.destroy_node()
        # Separate from the node: rclpy.init() lands before the node exists, so
        # a failure in between would otherwise leave ROS up and never shut down.
        if started.get("ros"):
            rclpy.shutdown()


def wait_for_node(rclpy, timeout=5.0, sleep=time.sleep):
    """Block until the spun node is visible in the global executor."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if rclpy.get_global_executor().get_nodes():
            return True
        sleep(0.01)
    raise RuntimeError(
        f"node did not join the global executor within {timeout}s; GEAR's simulator "
        "indexes it unguarded and would die with IndexError"
    )


MULTICAST_DEFAULT = 'multicast="default"'
MULTICAST_FORCED = 'multicast="true"'


def channel_patcher(g1_env, channel):
    """Three patches GEAR needs to share one DDS domain with rclpy.

    `init_channel` is wrapped on `g1_env`, not on `simulator_factory`: `g1_env`
    imported the name, so rebinding the source module rebinds a copy nobody
    reads. That wrap fixes the order.

    `ChannelFactory.Init` is wrapped to become idempotent, which fixes the
    count. GEAR initialises the channel more than once -- `g1_env`, then
    `base_sim` again when the simulator starts -- and `ChannelFactory` is a
    singleton whose `Init` is not guarded, so the second call re-runs
    `Domain(id, config)` on a domain that now exists and returns False. The
    class is patched rather than `ChannelFactoryInitialize`, because every
    caller imported that function by name but they all reach this one class.

    `ChannelConfigHasInterface` is forced to `multicast="true"`, which fixes
    discovery. The template is handed straight to `Domain(id, config)`, so
    `CYCLONEDDS_URI` cannot reach it, and `"default"` on `lo` resolves to off
    ("selected interface lo is not multicast-capable"): the sim and the bridge
    then never find each other. Forcing it here rather than setting the
    interface's MULTICAST flag keeps a rehearsal from needing root on every new
    machine. It is patched on `channel`, which imported the name, not on
    `channel_config`. On a real NIC `"true"` and `"default"` agree.

    Any of the three names moving upstream raises here, rather than silently
    leaving the failure it exists to avoid.
    """

    def patch(hook):
        real_init_channel = getattr(g1_env, "init_channel", None)
        if real_init_channel is None:
            raise RuntimeError(
                "g1_env has no init_channel to wrap: upstream moved it, and without "
                "the wrap rclpy takes the DDS domain first and the sim goes silent"
            )
        factory = getattr(channel, "ChannelFactory", None)
        if factory is None:
            raise RuntimeError(
                "unitree_sdk2py.core.channel has no ChannelFactory to guard: upstream "
                "moved it, and the simulator's second init would fail on the domain"
            )
        real_config = getattr(channel, "ChannelConfigHasInterface", None)
        if real_config is None or MULTICAST_DEFAULT not in real_config:
            raise RuntimeError(
                "unitree_sdk2py's ChannelConfigHasInterface is missing or no longer "
                f"carries {MULTICAST_DEFAULT}: without forcing multicast on, discovery "
                "over lo needs `ip link set lo multicast on` as root on every machine"
            )
        real_factory_init = factory.Init
        state = {}

        def guarded_init(self, id, networkInterface=None, qos=None):
            if state.get("ok"):
                return True
            state["ok"] = real_factory_init(self, id, networkInterface, qos)
            return state["ok"]

        factory.Init = guarded_init
        channel.ChannelConfigHasInterface = real_config.replace(
            MULTICAST_DEFAULT, MULTICAST_FORCED)
        g1_env.init_channel = lambda config: hook(real_init_channel, config)

        def restore():
            factory.Init = real_factory_init
            channel.ChannelConfigHasInterface = real_config
            g1_env.init_channel = real_init_channel

        return restore

    return patch


def main(argv=None):
    import rclpy
    import tyro
    from decoupled_wbc.control.envs.g1 import g1_env
    from decoupled_wbc.control.main.teleop import run_g1_control_loop
    from decoupled_wbc.control.main.teleop.configs.configs import ControlLoopConfig
    from unitree_sdk2py.core import channel

    from robot_bridge.adapters.gear_hands import install as install_hands
    from robot_bridge.adapters.gear_wbc import install
    from robot_bridge.cmd_client import RobotCmdClient

    config = tyro.cli(ControlLoopConfig, args=argv)
    # The whole domain-ordering fix depends on this. `messaging_backend`
    # defaults to "ros2", and upstream's main builds the loop manager before
    # the env (`run_g1_control_loop.py`), so the ros2 manager calls
    # `rclpy.init()` (`ros_utils.py`) before `init_channel` ever runs -- rmw
    # takes the domain first and the channel factory dies on it, deep inside
    # env construction where the message means nothing. The status topic this
    # node reads is a ZMQ port too.
    if config.messaging_backend != "zmq":
        print(
            f"--messaging-backend must be zmq here, not {config.messaging_backend!r}: "
            "the ros2 backend starts rclpy before GEAR's channel and the DDS domain "
            "goes to the wrong owner.",
            file=sys.stderr,
        )
        return 2
    if config.interface == "sim":
        print(
            "WARNING: --interface sim: MuJoCo listens on rt/lowcmd, which "
            "nothing writes here unless a G1_bridge runs against the sim.",
            file=sys.stderr,
        )
    run(
        config,
        rclpy,
        RobotCmdClient,
        install,
        run_g1_control_loop.main,
        channel_patcher(g1_env, channel),
        install_hands=install_hands,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
