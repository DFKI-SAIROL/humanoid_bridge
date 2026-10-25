#!/usr/bin/env python3
"""Release Unitree's own controller from /lowcmd, so the bridge can start.

Needs: `unitree_sdk2py`, and a NIC on the robot's subnet. No ROS.

The bridge refuses to start while anything else publishes /lowcmd: it retries
once a second logging "Detected N publishers on /lowcmd". `L2 + R2` on the
controller does the same thing as this script, and GEAR's BodyStateProcessor
does it on start when ENV_TYPE == real.

    tools/g1-release-lowcmd.py --iface enp4s0
    tools/g1-release-lowcmd.py --check          # report the mode, change nothing
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time

ATTEMPTS = 5


def _nics():
    try:
        out = subprocess.run(["ip", "-o", "link", "show"], capture_output=True,
                             text=True, check=True).stdout
    except (OSError, subprocess.CalledProcessError):
        return []
    return [line.split(": ")[1] for line in out.splitlines() if ": " in line]


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--iface", default=os.environ.get("BRIDGE_IFACE"),
                        help="DDS network interface (default $BRIDGE_IFACE)")
    parser.add_argument("--domain", type=int, default=0, help="DDS domain id (default 0)")
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--check", action="store_true",
                        help="report the current mode and exit without releasing")
    args = parser.parse_args(argv)

    if not args.iface:
        print("no interface: pass --iface or set BRIDGE_IFACE. Candidates:", file=sys.stderr)
        for nic in _nics():
            print(f"    {nic}", file=sys.stderr)
        return 2

    # Imported here so --help works without the SDK.
    try:
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
            MotionSwitcherClient,
        )
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    except ImportError:
        print(
            "unitree_sdk2py is not importable. Install it from "
            "https://github.com/unitreerobotics/unitree_sdk2_python, or run this "
            "with the interpreter that has it.",
            file=sys.stderr,
        )
        return 1

    if args.iface == "lo":
        # robot_sim: lo has no MULTICAST flag, so the SDK's multicast="default" finds no peer and every call times out.
        import unitree_sdk2py.core.channel as channel

        channel.ChannelConfigHasInterface = channel.ChannelConfigHasInterface.replace(
            'multicast="default"', 'multicast="true"'
        )
    ChannelFactoryInitialize(args.domain, args.iface)
    switcher = MotionSwitcherClient()
    switcher.SetTimeout(args.timeout)
    switcher.Init()

    print(f"before: {switcher.CheckMode()}")
    if args.check:
        return 0
    # Unitree's own example repeats ReleaseMode until CheckMode shows no mode.
    for _ in range(ATTEMPTS):
        print(f"release: {switcher.ReleaseMode()}")
        time.sleep(1.0)
        code, mode = after = switcher.CheckMode()
        print(f"after:  {after}")
        if code == 0 and isinstance(mode, dict) and not mode.get("name"):
            return 0
    print(f"release failed after {ATTEMPTS} tries: a motion mode still holds /lowcmd. "
          "Press L2 + R2 on the remote.", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
