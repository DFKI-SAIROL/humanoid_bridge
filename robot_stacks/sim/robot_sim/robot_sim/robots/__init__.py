"""One module per robot: SPEC (physics.RobotSpec) and Wire (its ROS side). Imported on demand."""

import importlib

NAMES = ("g1",)


def load(name):
    """The robot module for `name`; ValueError on an unknown one."""
    if name not in NAMES:
        raise ValueError(f"robot {name!r}: expected one of {NAMES}")
    return importlib.import_module(f"robot_sim.robots.{name}")
