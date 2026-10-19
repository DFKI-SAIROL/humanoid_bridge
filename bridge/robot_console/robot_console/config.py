"""The console's one config file: `console:` settings and `processes:`.

Pure: parses and validates, starts nothing. Schema: docs/robot_console.md.
"""

from __future__ import annotations

import dataclasses
import os
import re
from pathlib import Path
from typing import Mapping

import yaml

GROUPS = ("bring-up", "policy", "run")
SIGNALS = ("CTRL-C", "INT", "TERM", "HUP", "KILL")
# A param placeholder: "{name}", but not the "${VAR}" of an environment variable.
PLACEHOLDER = re.compile(r"(?<!\$)\{([A-Za-z_][A-Za-z0-9_]*)\}")
_REQUIRE = re.compile(r"^(armed|([A-Za-z_][A-Za-z0-9_-]*)\.(running|ready))$")


class ConfigError(ValueError):
    """The file does not describe a valid console setup."""


@dataclasses.dataclass(frozen=True)
class ParamSpec:
    name: str
    choices: tuple[str, ...] | None = None
    text: bool = False
    pattern: str | None = None
    default: str | None = None


@dataclasses.dataclass(frozen=True)
class ReadySpec:
    http: str | None = None
    log: str | None = None


@dataclasses.dataclass(frozen=True)
class StopSpec:
    signals: tuple[str, ...] = ("INT", "TERM", "KILL")
    grace_s: tuple[float, ...] = (10.0, 5.0)


@dataclasses.dataclass(frozen=True)
class PromptSpec:
    match: str
    answers: tuple[str, ...]


@dataclasses.dataclass(frozen=True)
class ProcessSpec:
    name: str
    argv: tuple[str, ...]
    label: str = ""
    group: str = "run"
    cwd: str | None = None  # unexpanded: ${VAR} and ~ expand at start
    env_file: str | None = None  # relative to cwd
    env: Mapping[str, str] = dataclasses.field(default_factory=dict)
    params: Mapping[str, ParamSpec] = dataclasses.field(default_factory=dict)
    tty: bool = False
    ready: ReadySpec | None = None
    stop: StopSpec = StopSpec()
    stop_argv: tuple[str, ...] | None = None
    commands_robot: bool = False
    requires: tuple[str, ...] = ()
    prompts: tuple[PromptSpec, ...] = ()


@dataclasses.dataclass(frozen=True)
class ConsoleSpec:
    bridge_config: Path
    robot: str = "robot"
    lowstate_topic: str = "/lowstate"
    lowstate_type: str = "unitree_hg/msg/LowState"
    hands: bool = False
    urdf: Path | None = None
    camera: str | None = None
    log_dir: Path = Path("~/.local/state/robot_console").expanduser()


@dataclasses.dataclass(frozen=True)
class Config:
    console: ConsoleSpec
    processes: tuple[ProcessSpec, ...]

    def process(self, name: str) -> ProcessSpec:
        for spec in self.processes:
            if spec.name == name:
                return spec
        raise KeyError(name)


def expand(text: str, environ: Mapping[str, str]) -> str:
    """`${VAR}`/`$VAR` from `environ` (unknown ones stay as written), then a leading `~`."""
    expanded = re.sub(
        r"\$\{([A-Za-z_][A-Za-z0-9_]*)\}|\$([A-Za-z_][A-Za-z0-9_]*)",
        lambda m: environ.get(m.group(1) or m.group(2), m.group(0)),
        text,
    )
    return os.path.expanduser(expanded)


# --- parsing ------------------------------------------------------------


def _str_list(value, where) -> tuple[str, ...]:
    if not isinstance(value, list) or not all(isinstance(v, (str, int, float)) for v in value):
        raise ConfigError(f"{where}: expected a list of strings")
    return tuple(str(v) for v in value)


def _keys(block, allowed, where):
    unknown = set(block) - set(allowed)
    if unknown:
        raise ConfigError(f"{where}: unknown key(s) {sorted(unknown)}")


def _regex(text, where):
    try:
        re.compile(text)
    except re.error as exc:
        raise ConfigError(f"{where}: bad regex {text!r}: {exc}") from None
    return text


def _param(name, block, where) -> ParamSpec:
    if not isinstance(block, dict):
        raise ConfigError(f"{where}: expected a mapping")
    _keys(block, ("choices", "text", "pattern", "default"), where)
    choices = _str_list(block["choices"], f"{where}.choices") if "choices" in block else None
    text = bool(block.get("text", False))
    if (choices is None) == (not text):
        raise ConfigError(f"{where}: give exactly one of choices or text: true")
    if choices is not None and not choices:
        raise ConfigError(f"{where}.choices: empty")
    pattern = block.get("pattern")
    if pattern is not None and not text:
        raise ConfigError(f"{where}.pattern: only for text params")
    if pattern is not None:
        _regex(str(pattern), f"{where}.pattern")
    default = None if block.get("default") is None else str(block["default"])
    if default is not None and choices is not None and default not in choices:
        raise ConfigError(f"{where}.default: {default!r} is not one of the choices")
    return ParamSpec(name, choices, text, None if pattern is None else str(pattern), default)


def _process(name, block) -> ProcessSpec:
    where = f"processes.{name}"
    if not isinstance(block, dict):
        raise ConfigError(f"{where}: expected a mapping")
    _keys(
        block,
        (
            "label",
            "group",
            "cwd",
            "env_file",
            "env",
            "argv",
            "params",
            "tty",
            "ready",
            "stop",
            "stop_argv",
            "commands_robot",
            "requires",
            "prompts",
        ),
        where,
    )
    if "argv" not in block:
        raise ConfigError(f"{where}: argv is required")
    argv = _str_list(block["argv"], f"{where}.argv")
    if not argv:
        raise ConfigError(f"{where}.argv: empty")

    group = block.get("group", "run")
    if group not in GROUPS:
        raise ConfigError(f"{where}.group: {group!r}, expected one of {GROUPS}")

    env = block.get("env") or {}
    if not isinstance(env, dict):
        raise ConfigError(f"{where}.env: expected a mapping")
    env = {str(k): str(v) for k, v in env.items()}

    raw_params = block.get("params") or {}
    if not isinstance(raw_params, dict):
        raise ConfigError(f"{where}.params: expected a mapping")
    params = {str(k): _param(str(k), v, f"{where}.params.{k}") for k, v in raw_params.items()}

    used = set()
    for element in argv:
        names = PLACEHOLDER.findall(element)
        for param in names:
            if param not in params:
                continue  # a literal brace, e.g. inside a bash -c script
            used.add(param)
            if params[param].text and element != "{" + param + "}":
                raise ConfigError(
                    f"{where}.argv: text param {param!r} must be a whole element, "
                    f"not part of {element!r}"
                )
    unused = set(params) - used
    if unused:
        raise ConfigError(f"{where}.params: {sorted(unused)} never appear in argv")

    tty = bool(block.get("tty", False))

    ready = None
    if block.get("ready") is not None:
        r = block["ready"]
        if not isinstance(r, dict) or len(r) != 1 or not set(r) <= {"http", "log"}:
            raise ConfigError(f"{where}.ready: expected {{http: url}} or {{log: regex}}")
        if "log" in r:
            ready = ReadySpec(log=_regex(str(r["log"]), f"{where}.ready.log"))
        else:
            ready = ReadySpec(http=str(r["http"]))

    stop = StopSpec()
    if block.get("stop") is not None:
        s = block["stop"]
        if not isinstance(s, dict):
            raise ConfigError(f"{where}.stop: expected a mapping")
        _keys(s, ("signals", "grace_s"), f"{where}.stop")
        signals = _str_list(s.get("signals", list(stop.signals)), f"{where}.stop.signals")
        bad = [sig for sig in signals if sig not in SIGNALS]
        if bad or not signals:
            raise ConfigError(f"{where}.stop.signals: {bad or 'empty'}; allowed {SIGNALS}")
        if "CTRL-C" in signals and not tty:
            raise ConfigError(f"{where}.stop.signals: CTRL-C needs tty: true")
        grace = s.get("grace_s", [10.0] * (len(signals) - 1))
        if not isinstance(grace, list) or len(grace) != len(signals) - 1:
            raise ConfigError(f"{where}.stop.grace_s: one value per signal but the last")
        stop = StopSpec(signals, tuple(float(g) for g in grace))

    stop_argv = None
    if block.get("stop_argv") is not None:
        stop_argv = _str_list(block["stop_argv"], f"{where}.stop_argv")

    requires = _str_list(block.get("requires", []), f"{where}.requires")
    for cond in requires:
        if not _REQUIRE.match(cond):
            raise ConfigError(
                f"{where}.requires: {cond!r}, expected armed, <name>.running or <name>.ready"
            )

    prompts = []
    for i, p in enumerate(block.get("prompts") or []):
        pw = f"{where}.prompts[{i}]"
        if not isinstance(p, dict) or set(p) != {"match", "answers"}:
            raise ConfigError(f"{pw}: expected {{match, answers}}")
        prompts.append(PromptSpec(_regex(str(p["match"]), pw), _str_list(p["answers"], pw)))
    if prompts and not tty:
        raise ConfigError(f"{where}.prompts: need tty: true")

    return ProcessSpec(
        name=name,
        argv=argv,
        label=str(block.get("label", name)),
        group=group,
        cwd=None if block.get("cwd") is None else str(block["cwd"]),
        env_file=None if block.get("env_file") is None else str(block["env_file"]),
        env=env,
        params=params,
        tty=tty,
        ready=ready,
        stop=stop,
        stop_argv=stop_argv,
        commands_robot=bool(block.get("commands_robot", False)),
        requires=requires,
        prompts=tuple(prompts),
    )


def _console(block, environ, base: Path) -> ConsoleSpec:
    if not isinstance(block, dict):
        raise ConfigError("console: expected a mapping")
    _keys(
        block,
        ("robot", "bridge_config", "lowstate", "hands", "urdf", "camera", "log_dir"),
        "console",
    )
    if "bridge_config" not in block:
        raise ConfigError("console.bridge_config is required: the motor table comes from it")

    def path(value):
        p = Path(expand(str(value), environ))
        return p if p.is_absolute() else (base / p)

    lowstate = block.get("lowstate") or {}
    if not isinstance(lowstate, dict) or not set(lowstate) <= {"topic", "type"}:
        raise ConfigError("console.lowstate: expected {topic, type}")
    defaults = ConsoleSpec(bridge_config=Path("."))
    return ConsoleSpec(
        bridge_config=path(block["bridge_config"]),
        robot=str(block.get("robot", defaults.robot)),
        lowstate_topic=str(lowstate.get("topic", defaults.lowstate_topic)),
        lowstate_type=str(lowstate.get("type", defaults.lowstate_type)),
        hands=bool(block.get("hands", False)),
        urdf=None if block.get("urdf") is None else path(block["urdf"]),
        camera=None if block.get("camera") is None else expand(str(block["camera"]), environ),
        log_dir=path(block["log_dir"]) if block.get("log_dir") else defaults.log_dir,
    )


def parse(document, environ: Mapping[str, str] | None = None, base: Path = Path(".")) -> Config:
    """A parsed YAML document -> Config. Relative console paths resolve against `base`."""
    environ = os.environ if environ is None else environ
    if not isinstance(document, dict):
        raise ConfigError("expected a mapping with console: and processes:")
    _keys(document, ("console", "processes"), "top level")
    console = _console(document.get("console"), environ, base)

    raw = document.get("processes") or {}
    if not isinstance(raw, dict):
        raise ConfigError("processes: expected a mapping of name -> entry")
    processes = tuple(_process(str(name), block) for name, block in raw.items())

    names = {p.name for p in processes}
    for spec in processes:
        for cond in spec.requires:
            m = _REQUIRE.match(cond)
            if m.group(2) is not None and m.group(2) not in names:
                raise ConfigError(f"processes.{spec.name}.requires: no process {m.group(2)!r}")
            if m.group(2) == spec.name:
                raise ConfigError(f"processes.{spec.name}.requires: requires itself")
    return Config(console=console, processes=processes)


def load(path, environ: Mapping[str, str] | None = None) -> Config:
    """Read and validate a console config file."""
    path = Path(path)
    with open(path) as handle:
        document = yaml.safe_load(handle)
    return parse(document, environ, base=path.resolve().parent)


def read_env_file(path: Path) -> dict[str, str]:
    """KEY=VALUE lines; `#` comments, blank lines and an `export ` prefix allowed."""
    values: dict[str, str] = {}
    for line in Path(path).read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("export "):
            line = line[len("export ") :].lstrip()
        key, sep, value = line.partition("=")
        if not sep or not re.match(r"^[A-Za-z_][A-Za-z0-9_]*$", key.strip()):
            continue
        value = value.strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in "'\"":
            value = value[1:-1]
        values[key.strip()] = value
    return values
