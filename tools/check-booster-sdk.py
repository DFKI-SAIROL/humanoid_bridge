#!/usr/bin/env python3
"""Checks bridge/robot_bridge/include/booster_mode.hpp against a Booster SDK clone.

    tools/check-booster-sdk.py <booster_robotics_sdk clone>

Exit 0: RobotMode, LocoApiId::kChangeMode and ChangeModeParameter's key match; 1: one differs.
"""

import re
import sys
from pathlib import Path

HEADER = Path(__file__).resolve().parents[1] / "bridge/robot_bridge/include/booster_mode.hpp"


def enum(text, name):
    """{member: value} of `enum class <name> { ... }`, members without their k prefix."""
    body = re.search(rf"enum class {name}(?:\s*:\s*\w+)?\s*{{(.*?)}}", text, re.S).group(1)
    return {m: int(v) for m, v in re.findall(r"k?(\w+)\s*=\s*(-?\d+)", body)}


def main(sdk):
    sdk = Path(sdk)
    shared = (sdk / "include/booster/robot/common/robot_shared.hpp").read_text()
    loco = (sdk / "include/booster/robot/b1/b1_loco_api.hpp").read_text()
    ours = HEADER.read_text()
    problems = []
    theirs_mode, our_mode = enum(shared, "RobotMode"), enum(ours, "RobotMode")
    for member, value in our_mode.items():
        if theirs_mode.get(member) != value:
            sdk_value = theirs_mode.get(member)
            problems.append(f"RobotMode::{member} = {value}, the SDK says {sdk_value}")
    change = enum(loco, "LocoApiId").get("ChangeMode")
    ours_change = int(re.search(r"API_CHANGE_MODE = (\d+)", ours).group(1))
    if change != ours_change:
        problems.append(f"API_CHANGE_MODE = {ours_change}, the SDK says {change}")
    if not re.search(r'class ChangeModeParameter.*?json\["mode"\]', loco, re.S):
        problems.append('ChangeModeParameter no longer writes json["mode"]')
    for problem in problems:
        print(f"FAIL  {problem}")
    if not problems:
        print(f"ok    {HEADER.name} matches {sdk}")
    return 1 if problems else 0


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    sys.exit(main(sys.argv[1]))
