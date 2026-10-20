#!/usr/bin/env python3
"""Compare the G1 body joint limits in the bridge config with one or more URDFs.

    tools/check-g1-limits.py --urdf a.urdf [--urdf b.urdf] [--config G1_config.yaml]

Exit 0: all match. 1: a mismatch. 2: usage error or unreadable file.
"""

from __future__ import annotations

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

DEFAULT_CONFIG = (
    Path(__file__).resolve().parent.parent / "bridge/robot_bridge/params/G1_config.yaml"
)
TOL = 1e-3
# config field -> URDF <limit> attribute
FIELDS = (("q_min", "lower"), ("q_max", "upper"), ("dq_limit", "velocity"), ("tau_limit", "effort"))
# (joint, field) -> the value the config must hold, whatever the URDF says.
INTENDED = {
    ("L_LEG_HIP_ROLL", "dq_limit"): 20.0,  # rev 1.0 URDF value; the original says 32
    ("R_LEG_HIP_ROLL", "dq_limit"): 20.0,  # rev 1.0 URDF value; the original says 32
}
_SIDES = (("L_", "left_"), ("R_", "right_"))


class UsageError(Exception):
    """A file is missing, unreadable or malformed."""


def urdf_name(config_name: str) -> str:
    """Config name -> URDF joint name: `L_LEG_HIP_PITCH` -> `left_hip_pitch_joint`."""
    for prefix, side in _SIDES:
        if config_name.startswith(prefix):
            rest = config_name[len(prefix) :].lower()
            if rest.startswith("leg_"):
                rest = rest[len("leg_") :]
            return f"{side}{rest}_joint"
    return f"{config_name.lower()}_joint"


def load_config(path) -> dict:
    """Body joint name -> {field: value}, in `joint_names` order."""
    try:
        with open(path) as handle:
            document = yaml.safe_load(handle)
        (node,) = document.values()
        params = node["ros__parameters"]
        joints = {}
        for name in params["joint_names"]:
            joints[name] = {field: float(params[name][field]) for field, _ in FIELDS}
    except (OSError, yaml.YAMLError) as exc:
        raise UsageError(f"{path}: {exc}") from exc
    except (AttributeError, KeyError, TypeError, ValueError) as exc:
        raise UsageError(f"{path}: not a bridge config ({exc!r})") from exc
    return joints


def load_urdf(path) -> dict:
    """URDF joint name -> {attribute: value}; attributes absent from <limit> are left out."""
    try:
        root = ET.parse(path).getroot()
    except (OSError, ET.ParseError) as exc:
        raise UsageError(f"{path}: {exc}") from exc
    joints = {}
    for joint in root.iter("joint"):
        limit = joint.find("limit")
        attrs = {} if limit is None else limit.attrib
        try:
            joints[joint.get("name")] = {
                attr: float(attrs[attr]) for _, attr in FIELDS if attr in attrs
            }
        except ValueError as exc:
            raise UsageError(f"{path}: joint {joint.get('name')}: {exc}") from exc
    return joints


def compare(config: dict, urdf: dict) -> tuple[list[str], int]:
    """(mismatch lines, number of fields checked) for one URDF."""
    problems = []
    checked = 0
    for name, limits in config.items():
        joint = urdf_name(name)
        if joint not in urdf:
            problems.append(f"{name}: no joint {joint} in the urdf")
            continue
        for field, attr in FIELDS:
            theirs = urdf[joint].get(attr, 0.0)
            if theirs == 0.0:
                continue
            checked += 1
            ours = limits[field]
            intended = INTENDED.get((name, field))
            if intended is not None:
                if ours != intended:
                    problems.append(f"{name} {field}: config {ours:g} intended {intended:g}")
            elif abs(ours - theirs) > TOL:
                problems.append(f"{name} {field}: config {ours:g} urdf {theirs:g}")
    return problems, checked


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--urdf", action="append", required=True, help="URDF file (repeatable)")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG), help="bridge config yaml")
    args = parser.parse_args(argv)

    try:
        config = load_config(args.config)
        urdfs = [(path, load_urdf(path)) for path in args.urdf]
    except UsageError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    failed = False
    for path, urdf in urdfs:
        problems, checked = compare(config, urdf)
        if problems:
            failed = True
            print(f"FAIL {path}:")
            for line in problems:
                print(f"  {line}")
        else:
            print(f"ok   {path}: {len(config)} joints, {checked} fields checked")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
