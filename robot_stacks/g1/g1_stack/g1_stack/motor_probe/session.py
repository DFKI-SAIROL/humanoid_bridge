"""State core of the per-motor jog panel: desired q, per-motor gains, liveness.

Pure python on purpose -- no ROS, no I/O, no clock. The caller publishes what
`command()` returns and supplies `now` to the heartbeat calls.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from g1_stack.motor_probe.catalog import Catalog, MotorEntry

BODY = "body"
LEFT_HAND = "left_hand"
RIGHT_HAND = "right_hand"

# Every group is commanded every cycle: an unfed hand trips the bridge watchdog.
GROUPS = (BODY, LEFT_HAND, RIGHT_HAND)


def _clamp(value: float, low: float, high: float) -> float:
    """`value` pulled into [low, high]."""
    if value < low:
        return low
    return high if value > high else value


class ProbeSession:
    """Desired joint positions and gains for one motor-probe panel."""

    def __init__(self, catalog: Catalog, limp: bool = False, heartbeat_timeout: float = 0.5):
        self._limp = bool(limp)
        self._heartbeat_timeout = float(heartbeat_timeout)
        self._last_beat: float | None = None
        self._selected: str | None = None
        self._by_name: dict[str, MotorEntry] = {}
        self._groups: dict[str, list[MotorEntry]] = {group: [] for group in GROUPS}
        for entry in catalog.entries:
            if entry.group not in self._groups:
                raise ValueError(f"motor {entry.name!r} has unknown group {entry.group!r}")
            if entry.name in self._by_name:
                raise ValueError(f"duplicate motor name {entry.name!r}")
            self._by_name[entry.name] = entry
            self._groups[entry.group].append(entry)
        for group, entries in self._groups.items():
            entries.sort(key=lambda entry: entry.index)
            # Index doubles as the slot in the command vector, so it must be dense.
            if [entry.index for entry in entries] != list(range(len(entries))):
                raise ValueError(f"group {group!r} indices are not 0..{len(entries) - 1}")
        self._desired: dict[str, list[float]] = {
            group: [0.0] * len(entries) for group, entries in self._groups.items()
        }

    def latch(self, measured: dict) -> None:
        """Seed desired q from `measured` (group -> floats); omitted groups keep theirs."""
        for group, values in measured.items():
            entries = self._entries(group)
            values = [float(value) for value in values]
            if len(values) != len(entries):
                raise ValueError(f"{group}: got {len(values)} values, want {len(entries)}")
            self._desired[group] = [
                _clamp(value, entry.q_min, entry.q_max) for value, entry in zip(values, entries)
            ]

    def select(self, name: str) -> None:
        """Make `name` the motor that stays powered in limp mode."""
        self._selected = self._entry(name).name

    def selected(self) -> str | None:
        """The selected motor name, or None."""
        return self._selected

    def set_target(self, name: str, q: float) -> float:
        """Store `q` clamped to that motor's travel, and return the clamped value."""
        entry = self._entry(name)
        value = _clamp(float(q), float(entry.q_min), float(entry.q_max))
        self._desired[entry.group][entry.index] = value
        return value

    def set_limp(self, limp: bool) -> None:
        """Zero the gains of every motor but the selected one."""
        self._limp = bool(limp)

    def limp(self) -> bool:
        """Whether idle motors are being left slack."""
        return self._limp

    def command(self, group: str) -> dict:
        """{"q", "kp", "kd"} as plain float lists, one entry per motor in `group`."""
        entries = self._entries(group)
        desired = self._desired[group]
        q: list[float] = []
        kp: list[float] = []
        kd: list[float] = []
        for entry in entries:
            powered = not self._limp or entry.name == self._selected
            q.append(float(desired[entry.index]))
            kp.append(float(entry.kp) if powered else 0.0)
            kd.append(float(entry.kd) if powered else 0.0)
        return {"q": q, "kp": kp, "kd": kd}

    def note_heartbeat(self, now: float) -> None:
        """Record a heartbeat stamped `now`."""
        self._last_beat = float(now)

    def is_alive(self, now: float) -> bool:
        """True while the last heartbeat is within `heartbeat_timeout` of `now`."""
        if self._last_beat is None:
            return False
        return float(now) - self._last_beat <= self._heartbeat_timeout

    def _entry(self, name: str) -> MotorEntry:
        try:
            return self._by_name[name]
        except KeyError:
            raise ValueError(f"unknown motor {name!r}") from None

    def _entries(self, group: str) -> list[MotorEntry]:
        try:
            return self._groups[group]
        except KeyError:
            raise ValueError(f"unknown group {group!r}") from None
