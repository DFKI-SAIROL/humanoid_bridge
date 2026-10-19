"""The commandable motors as one flat table, read from the bridge's own config.

Body motors come from `joint_names`; the Dex3 hands from `*_hand_joint_names`
when hands are on. Two namings: the config's (`L_ELBOW`) and the URDF's
(`left_elbow_joint`). Every entry carries both.
"""

from __future__ import annotations

import dataclasses
from pathlib import Path

import yaml

BODY = "body"
LEFT_HAND = "left_hand"
RIGHT_HAND = "right_hand"

GROUPS = (BODY, LEFT_HAND, RIGHT_HAND)
_LIMIT_FIELDS = ("q_min", "q_max", "dq_limit", "tau_limit", "kp", "kd")
_SIDES = (("L_", "left_"), ("R_", "right_"))


@dataclasses.dataclass(frozen=True)
class MotorEntry:
    """One motor: its config block, plus the URDF name for the same joint."""

    name: str
    group: str
    index: int
    q_min: float
    q_max: float
    dq_limit: float
    tau_limit: float
    kp: float
    kd: float
    urdf_joint: str


def urdf_name(config_name: str) -> str:
    """Config name -> URDF joint name: `L_LEG_HIP_PITCH` -> `left_hip_pitch_joint`."""
    for prefix, side in _SIDES:
        if config_name.startswith(prefix):
            rest = config_name[len(prefix) :].lower()
            # "leg" is the one config segment the URDF drops; "hand" it keeps.
            if rest.startswith("leg_"):
                rest = rest[len("leg_") :]
            return f"{side}{rest}_joint"
    return f"{config_name.lower()}_joint"


class Catalog:
    """The motor table: 29 body motors in command order, then left then right hand."""

    def __init__(self, entries):
        self.entries = tuple(entries)
        self._by_name = {entry.name: entry for entry in self.entries}
        self._by_urdf = {entry.urdf_joint: entry for entry in self.entries}
        if len(self._by_name) != len(self.entries):
            raise ValueError("duplicate motor name in config")
        if len(self._by_urdf) != len(self.entries):
            raise ValueError("two motors map to one URDF joint name")
        for group in GROUPS:
            found = sorted(entry.index for entry in self.entries if entry.group == group)
            if found != list(range(len(found))):
                raise ValueError(f"{group}: expected indices 0..{len(found) - 1}, got {found}")
        if not self.group(BODY):
            raise ValueError("no body motors")

    def by_name(self, name: str) -> MotorEntry:
        """Look up by config name. KeyError if there is no such motor."""
        return self._by_name[name]

    def by_urdf(self, urdf_joint: str) -> MotorEntry:
        """Look up by URDF joint name. KeyError if there is no such motor."""
        return self._by_urdf[urdf_joint]

    def group(self, group: str) -> tuple:
        """One group's motors, ordered by command index."""
        if group not in GROUPS:
            raise KeyError(f"no such group: {group!r}")
        return tuple(sorted((e for e in self.entries if e.group == group), key=lambda e: e.index))

    def groups(self) -> tuple:
        """The groups that have motors, in GROUPS order."""
        return tuple(g for g in GROUPS if any(e.group == g for e in self.entries))

    def to_json(self) -> list:
        """Every entry as a plain dict, JSON-safe."""
        return [dataclasses.asdict(entry) for entry in self.entries]


def _entry(params, name, group, expected_index=None):
    block = params.get(name)
    if not isinstance(block, dict):
        raise ValueError(f"{name}: no parameter block in config")
    missing = [field for field in ("idx", *_LIMIT_FIELDS) if field not in block]
    if missing:
        raise ValueError(f"{name}: missing {missing}")
    index = int(block["idx"])
    if expected_index is not None and index != expected_index:
        raise ValueError(f"{name}: idx {index}, but listed at position {expected_index}")
    limits = {field: float(block[field]) for field in _LIMIT_FIELDS}
    return MotorEntry(name=name, group=group, index=index, urdf_joint=urdf_name(name), **limits)


def _names(params, key):
    names = params.get(key)
    if not isinstance(names, list) or not names:
        raise ValueError(f"{key}: expected a non-empty list of names")
    return names


def load_catalog(config_path, hands: bool = False) -> Catalog:
    """Parse a bridge config into a Catalog. Raises ValueError if the table is malformed."""
    path = Path(config_path)
    with open(path) as handle:
        document = yaml.safe_load(handle)
    try:
        params = document["robot_bridge"]["ros__parameters"]
    except (KeyError, TypeError) as exc:
        raise ValueError(f"{path}: no robot_bridge.ros__parameters") from exc

    entries = [
        _entry(params, name, BODY, expected_index=i)
        for i, name in enumerate(_names(params, "joint_names"))
    ]
    if hands:
        for key, group in (
            ("left_hand_joint_names", LEFT_HAND),
            ("right_hand_joint_names", RIGHT_HAND),
        ):
            entries += [_entry(params, name, group) for name in _names(params, key)]
    return Catalog(entries)
