"""Puts package sources ahead of the colcon install space.

Without this, `pytest` tests the last `colcon build` rather than the working tree.
"""

import pathlib
import sys

_ROOT = pathlib.Path(__file__).parent

for _rel in (
    "bridge/robot_bridge",
    "robot_stacks/g1/g1_stack",
    "robot_stacks/g1/g1_camera",
):
    _src = str(_ROOT / _rel)
    if _src not in sys.path:
        sys.path.insert(0, _src)
