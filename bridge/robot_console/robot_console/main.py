"""Robot console: one web page to operate a robot through the bridge.

**This node commands.** Arm, ready, jog and the processes it starts all move
the robot. The remote stays the primary e-stop.

    ros2 run robot_console robot_console --config <file> --host <lan-ip>

Design and rules: docs/robot_console.md.
"""

from __future__ import annotations

import argparse
import ipaddress
import os
import sys
import threading
from pathlib import Path

from . import auth
from . import server as server_mod
from .catalog import load_catalog
from .config import ConfigError, load
from .ros import PERIOD, EstopLink, RosLink
from .service import ConsoleService, model_source
from .session import ProbeSession
from .supervisor import Supervisor


def _check_host(host: str) -> None:
    """Refuse the wildcard: binding every NIC puts the page on the robot network too."""
    try:
        address = ipaddress.ip_address(host)
    except ValueError:
        return  # a hostname: the operator chose it
    if address.is_unspecified:
        sys.exit(f"--host {host} binds every interface, the robot network too; give the lab-LAN IP")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", required=True, help="console YAML: console: + processes:")
    parser.add_argument("--host", default="127.0.0.1", help="bind address (default localhost)")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--token-file", default=str(auth.DEFAULT_TOKEN_FILE))
    parser.add_argument(
        "--vendor",
        default=str(server_mod.DEFAULT_VENDOR_ROOT),
        help="three.js and urdf-loader; scripts/fetch_assets.sh fills it",
    )
    args = parser.parse_args(argv)
    _check_host(args.host)
    # Lets a config name paths beside itself, e.g. its own repo: ${ROBOT_CONSOLE_CONFIG_DIR}/../..
    os.environ.setdefault("ROBOT_CONSOLE_CONFIG_DIR", str(Path(args.config).resolve().parent))

    try:
        config = load(args.config)
        catalog = load_catalog(config.console.bridge_config, hands=config.console.hands)
    except (ConfigError, ValueError, OSError) as exc:
        sys.exit(f"config: {exc}")
    token = auth.load_or_create_token(Path(args.token_file).expanduser())

    import rclpy
    from rclpy.node import Node

    rclpy.init()
    node = Node("robot_console")

    holder = {}
    supervisor = Supervisor(
        config.processes,
        config.console.log_dir,
        conditions=lambda: {"armed": holder["service"].armed()},
    )
    estop = EstopLink(
        on_reply=lambda mode, ok, message: holder["service"].note(
            "warn" if ok else "bad", f"E-STOP {mode.upper()} reply: {message}"
        )
    )
    service = ConsoleService(config, catalog, ProbeSession(catalog), supervisor, estop)
    holder["service"] = service
    ros = RosLink(node, config, catalog, service)

    urdf = config.console.urdf
    if urdf is not None and not urdf.is_file():
        print(f"no URDF at {urdf}: the page falls back to the list view", file=sys.stderr)
    elif urdf is not None:
        print(f"3D model: {urdf.name} · {model_source(urdf)}", file=sys.stderr)
    httpd = server_mod.serve(
        service,
        token,
        host=args.host,
        port=args.port,
        assets_root=None if urdf is None else urdf.parent,
        vendor_root=Path(args.vendor).expanduser(),
        camera_url=config.console.camera,
    )
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    print(f"robot console on http://{args.host}:{args.port}/?token={token}", flush=True)

    # One thread spins the main node: RobotCmdClient's service calls spin it too,
    # and two spinners at once deadlock.
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=PERIOD)
            ros.handle_requests()
            ros.publish()
    except KeyboardInterrupt:
        pass
    finally:
        print("stopping processes...", flush=True)
        supervisor.shutdown()
        httpd.shutdown()
        estop.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
