"""Runs the config's processes: spawn, output, prompts, readiness, stop.

One lock guards all state; every public method is thread-safe. Schema: docs/robot_console.md.
"""

from __future__ import annotations

import codecs
import dataclasses
import datetime
import errno
import os
import pty
import queue
import re
import select
import shlex
import shutil
import signal
import subprocess
import sys
import threading
import time
import urllib.request
from collections import deque
from collections.abc import Callable, Mapping, Sequence
from pathlib import Path
from typing import TextIO

from .config import PLACEHOLDER, ProcessSpec, expand, read_env_file

RING = 200  # log lines kept in memory per process
RUNNING = ("starting", "ready", "prompt", "stopping")
EXTERNAL = "external: started outside the console"
SPAWN_WAIT_S = 5.0
LAST_WAIT_S = 2.0  # wait after the last stop signal
DRAIN_S = 1.0  # output still read after the exit, while a grandchild holds the pipe
STOP_ARGV_TIMEOUT_S = 30.0
PARTIAL_MAX = 8192  # a longer unfinished line is logged as a line
_HOLDS = {
    "running": ("starting", "ready", "prompt", "external"),
    "ready": ("ready", "external"),
}
# Makes the pty the controlling terminal, so a written ^C becomes SIGINT; then execs argv.
_CTTY = (
    "import fcntl, os, sys, termios; "
    "fcntl.ioctl(0, termios.TIOCSCTTY, 0); "
    "os.execvp(sys.argv[1], sys.argv[1:])"
)
_NO_PROXY = urllib.request.build_opener(urllib.request.ProxyHandler({}))


class _Refused(Exception):
    """A start refusal; the message is the reason."""


class _Job:
    """A start handed to the spawn thread."""

    def __init__(self, name: str, params: dict):
        self.name = name
        self.params = params
        self.result: tuple[bool, str] = (False, "not run")
        self.done = threading.Event()
        self.cancelled = False


@dataclasses.dataclass(eq=False)
class _Run:
    """One start of a process."""

    popen: subprocess.Popen
    fd: int | None  # pipe or pty master; only the reader closes it
    tty: bool
    cwd: str | None
    env: dict[str, str]
    stop_argv: list[str] | None
    ready: bool = False  # the ready check passed
    exited: bool = False
    exited_at: float = 0.0
    stop_requested: bool = False
    stop_done: bool = False
    decoder: codecs.IncrementalDecoder = dataclasses.field(
        default_factory=lambda: codecs.getincrementaldecoder("utf-8")(errors="replace")
    )


@dataclasses.dataclass(eq=False)
class _Proc:
    """One configured process across its starts."""

    spec: ProcessSpec
    prompts: list  # (compiled match, PromptSpec)
    ready_log: re.Pattern | None
    state: str = "stopped"
    run: _Run | None = None
    exit_code: int | None = None
    prompt: dict | None = None
    seq: int = 0
    ring: deque = dataclasses.field(default_factory=lambda: deque(maxlen=RING))
    partial: str = ""
    logf: TextIO | None = None
    log_path: str | None = None
    started_at: str | None = None
    warned: bool = False


class Supervisor:
    """Starts, watches and stops the configured processes."""

    def __init__(
        self,
        specs: Sequence[ProcessSpec],
        log_dir: Path,
        conditions: Callable[[], Mapping[str, bool]] | None = None,
        environ: Mapping[str, str] | None = None,
        ready_period: float = 0.5,
    ):
        self._procs: dict[str, _Proc] = {}
        for spec in specs:
            if spec.name in self._procs:
                raise ValueError(f"duplicate process {spec.name!r}")
            ready_log = spec.ready.log if spec.ready else None
            self._procs[spec.name] = _Proc(
                spec,
                prompts=[(re.compile(p.match), p) for p in spec.prompts],
                ready_log=None if ready_log is None else re.compile(ready_log),
            )
        self._log_dir = Path(log_dir).expanduser()
        self._conditions = conditions
        self._environ = environ
        self._ready_period = ready_period
        self._setpriv = shutil.which("setpriv")
        self._lock = threading.Lock()
        self._changed = threading.Condition(self._lock)
        self._closing = False
        self._jobs: queue.Queue = queue.Queue()
        self._halt = threading.Event()
        self._worker = threading.Thread(target=self._work, name="supervisor-spawn", daemon=True)
        self._monitor = threading.Thread(target=self._watch, name="supervisor-ready", daemon=True)
        self._worker.start()
        self._monitor.start()

    # --- public -----------------------------------------------------------

    def start(self, name: str, params: Mapping[str, str] | None = None) -> tuple[bool, str]:
        """Starts `name` on the spawn thread; waits up to 5 s for the result."""
        if params is not None and not isinstance(params, Mapping):
            return False, "params: expected a mapping"
        with self._lock:
            if self._closing:
                return False, "shutting down"
        job = _Job(name, dict(params or {}))
        self._jobs.put(job)
        if not job.done.wait(SPAWN_WAIT_S):
            job.cancelled = True  # skipped if the spawn thread has not begun it
            return False, "the spawn thread did not answer within 5 s"
        return job.result

    def stop(self, name: str) -> tuple[bool, str]:
        """Begins the stop sequence on its own thread; returns at once."""
        with self._lock:
            proc = self._procs.get(name)
            if proc is None:
                return False, f"unknown process {name!r}"
            if proc.state == "external":
                return False, EXTERNAL
            if proc.state == "stopping":
                return False, "already stopping"
            if proc.state not in RUNNING:
                return False, "not running"
            self._begin_stop_locked(proc)
        return True, "stopping"

    def answer(self, name: str, answer: str) -> tuple[bool, str]:
        """Writes `answer` and a newline to the pty of a pending prompt."""
        with self._lock:
            proc = self._procs.get(name)
            if proc is None:
                return False, f"unknown process {name!r}"
            if proc.state != "prompt" or proc.prompt is None:
                return False, "no prompt pending"
            if answer not in proc.prompt["answers"]:
                return False, f"answer must be one of {proc.prompt['answers']}"
            run = proc.run
            if run is None or run.fd is None:
                return False, "the terminal is closed"
            try:
                os.write(run.fd, (answer + "\n").encode())
            except OSError as exc:
                return False, f"write failed: {exc}"
            proc.prompt = None
            # the prompt line becomes a log line, so it cannot match again before the echo
            if proc.partial:
                self._emit_locked(proc, [proc.partial])
                proc.partial = ""
            proc.state = "ready" if run.ready else "starting"
        return True, "answered"

    def requires(self, name: str) -> list[tuple[str, bool]]:
        """Each `requires` condition of `name` and whether it holds now; KeyError if unknown."""
        conds = self._conds()
        with self._lock:
            return self._requires_locked(self._procs[name].spec, conds)

    def snapshot(self) -> list[dict]:
        """One dict per process, in spec order."""
        conds = self._conds()
        with self._lock:
            return [self._describe_locked(proc, conds) for proc in self._procs.values()]

    def log(self, name: str, since: int = 0) -> dict:
        """{"seq": last seq, "lines": complete lines with seq > since}; KeyError if unknown."""
        with self._lock:
            proc = self._procs[name]
            if since > proc.seq:
                since = 0  # the console restarted: seq began again
            return {"seq": proc.seq, "lines": [line for seq, line in proc.ring if seq > since]}

    def running_commands_robot(self) -> list[str]:
        """Names of commands_robot processes that run or stop now."""
        with self._lock:
            return [
                proc.spec.name
                for proc in self._procs.values()
                if proc.spec.commands_robot and proc.state in RUNNING
            ]

    def shutdown(self, timeout: float = 30.0) -> None:
        """Stops every running process in parallel, waits up to `timeout`, ends the threads."""
        deadline = time.monotonic() + timeout
        with self._lock:
            self._closing = True
            for proc in self._procs.values():
                if proc.state in ("starting", "ready", "prompt"):
                    self._begin_stop_locked(proc)
            self._changed.wait_for(
                lambda: not any(p.state in RUNNING for p in self._procs.values()),
                max(0.0, deadline - time.monotonic()),
            )
        self._halt.set()
        self._jobs.put(None)
        # trap: the spawn thread's exit sends TERM (pdeathsig) to every child still alive
        self._worker.join(max(1.0, deadline - time.monotonic()))
        self._monitor.join(2.0)
        with self._lock:
            for proc in self._procs.values():
                if proc.logf is not None:
                    proc.logf.close()
                    proc.logf = None

    # --- start (spawn thread) ---------------------------------------------

    def _work(self) -> None:
        """The spawn thread: runs start jobs in order until shutdown."""
        # trap: PR_SET_PDEATHSIG fires when the forking thread exits: only this thread spawns
        while True:
            job = self._jobs.get()
            if job is None:
                return
            if job.cancelled:
                continue
            try:
                job.result = self._start(job)
            except Exception as exc:  # this thread must live: its exit TERMs every child
                job.result = (False, f"internal error: {exc!r}")
            job.done.set()

    def _start(self, job: _Job) -> tuple[bool, str]:
        """Checks, plans and spawns one start."""
        conds = self._conds()
        with self._lock:
            proc = self._procs.get(job.name)
            if proc is None:
                return False, f"unknown process {job.name!r}"
            if self._closing:
                return False, "shutting down"
            if proc.state in RUNNING:
                return False, f"already {proc.state}"
            if proc.state == "external":
                return False, EXTERNAL
            for cond, ok in self._requires_locked(proc.spec, conds):
                if not ok:
                    return False, f"requires {cond}"
        spec = proc.spec
        try:
            values = _param_values(spec, job.params)
            cwd, env = self._environment(spec)
            argv = [_fill(expand(element, env), spec, values) for element in spec.argv]
        except _Refused as exc:
            return False, str(exc)
        stop_argv = None if spec.stop_argv is None else [expand(a, env) for a in spec.stop_argv]

        now = datetime.datetime.now().astimezone()
        try:
            logf, log_path = self._open_log(spec.name, now)
        except OSError as exc:
            return False, f"cannot open the log: {exc}"
        with self._lock:
            if proc.logf is not None:
                proc.logf.close()
            proc.logf, proc.log_path = logf, str(log_path)
            stamp = now.isoformat(timespec="seconds")
            self._emit_locked(proc, [f"=== start {stamp}: {shlex.join(argv)} ==="])
            if self._setpriv is None and not proc.warned:
                proc.warned = True
                warning = "=== warning: setpriv not found: this process can outlive the console ==="
                self._emit_locked(proc, [warning])

        command = list(argv)
        if spec.tty:
            command = [sys.executable, "-I", "-S", "-c", _CTTY, *command]
        if self._setpriv is not None:
            command = [self._setpriv, "--pdeathsig", "TERM", "--", *command]
        try:
            _check_program(argv[0], cwd, env)
            popen, fd = _spawn(command, cwd, env, spec.tty)
        except (OSError, ValueError, subprocess.SubprocessError) as exc:
            reason = f"exec failed: {exc}"
            with self._lock:
                self._emit_locked(proc, [f"=== {reason} ==="])
            return False, reason

        run = _Run(popen, fd, spec.tty, cwd, env, stop_argv, ready=spec.ready is None)
        with self._lock:
            proc.run = run
            proc.state = "ready" if run.ready else "starting"
            proc.exit_code = None
            proc.prompt = None
            proc.partial = ""
            proc.started_at = stamp
            for target, role in ((self._read, "read"), (self._wait, "wait")):
                threading.Thread(
                    target=target, args=(proc, run), name=f"{role}-{spec.name}", daemon=True
                ).start()
            if self._closing:
                self._begin_stop_locked(proc)
        return True, f"started, pid {popen.pid}"

    def _environment(self, spec: ProcessSpec) -> tuple[str | None, dict[str, str]]:
        """(cwd, env): base env, env_file keys the base lacks, then spec.env, each expanded."""
        env = dict(os.environ if self._environ is None else self._environ)
        if spec.env_file:
            # the env file lives in the cwd as the base environment expands it
            here = expand(spec.cwd, env) if spec.cwd else os.getcwd()
            if not os.path.isdir(here):
                raise _Refused(f"cwd not found: {here}")
            try:
                values = read_env_file(Path(here, expand(spec.env_file, env)))
            except (OSError, ValueError) as exc:
                raise _Refused(f"env_file: {exc}") from None
            for key, value in values.items():
                env.setdefault(key, value)
        for key, value in spec.env.items():
            env[key] = expand(value, env)
        cwd = os.path.abspath(expand(spec.cwd, env)) if spec.cwd else None
        if cwd is not None and not os.path.isdir(cwd):
            raise _Refused(f"cwd not found: {cwd}")
        return cwd, env

    def _open_log(self, name: str, now: datetime.datetime) -> tuple[TextIO, Path]:
        """<log_dir>/<date>/<name>-<HHMMSS>.log, opened for append."""
        folder = self._log_dir / now.strftime("%Y-%m-%d")
        folder.mkdir(parents=True, exist_ok=True)
        path = folder / f"{re.sub(r'[^A-Za-z0-9_.-]', '_', name)}-{now:%H%M%S}.log"
        return open(path, "a", encoding="utf-8"), path

    # --- output (reader thread per run) -------------------------------------

    def _read(self, proc: _Proc, run: _Run) -> None:
        """Feeds the child's output until EOF, or DRAIN_S after the exit; then closes the fd."""
        poller = select.poll()
        poller.register(run.fd, select.POLLIN)
        while True:
            if not poller.poll(100):
                if run.exited and time.monotonic() - run.exited_at > DRAIN_S:
                    break
                continue
            try:
                data = os.read(run.fd, 65536)
            except BlockingIOError:
                continue
            except OSError:
                break  # EIO: the pty's child side is closed
            if not data:
                break
            self._feed(proc, run, run.decoder.decode(data))
        self._feed(proc, run, run.decoder.decode(b"", final=True), eof=True)
        with self._lock:
            os.close(run.fd)
            run.fd = None

    def _feed(self, proc: _Proc, run: _Run, text: str, eof: bool = False) -> None:
        """Splits output into lines; checks readiness and prompts on the new ones."""
        text = text.replace("\r", "")
        with self._lock:
            lines = (proc.partial + text).split("\n")
            proc.partial = lines.pop()
            if (eof and proc.partial) or len(proc.partial) > PARTIAL_MAX:
                lines.append(proc.partial)
                proc.partial = ""
            self._emit_locked(proc, lines)
            if proc.run is run and not eof:
                self._match_locked(proc, run, lines)

    def _match_locked(self, proc: _Proc, run: _Run, lines: list[str]) -> None:
        """Log readiness and prompts against this read's lines and the unfinished one."""
        rx = proc.ready_log
        if rx is not None and not run.ready and proc.state in ("starting", "prompt"):
            if any(rx.search(line) for line in (*lines, proc.partial)):
                run.ready = True
                if proc.state == "starting":
                    proc.state = "ready"
        if not proc.spec.tty or proc.prompt is not None or proc.state not in ("starting", "ready"):
            return
        candidates = [proc.partial] if proc.partial else []
        if lines:
            candidates.append(lines[-1])
        for line in candidates:
            for match, prompt in proc.prompts:
                if match.search(line):
                    proc.prompt = {"text": line, "answers": list(prompt.answers)}
                    proc.state = "prompt"
                    return

    def _emit_locked(self, proc: _Proc, lines: list[str]) -> None:
        """Appends complete lines to the ring and the log file."""
        for line in lines:
            proc.seq += 1
            proc.ring.append((proc.seq, line))
        if lines and proc.logf is not None:
            try:
                proc.logf.write("".join(line + "\n" for line in lines))
                proc.logf.flush()
            except (OSError, ValueError):
                pass  # a full disk or a closed file: the ring still has the lines

    # --- exit and stop --------------------------------------------------------

    def _wait(self, proc: _Proc, run: _Run) -> None:
        """Records the exit: exited, or stopped once a requested stop sequence is done."""
        code = run.popen.wait()
        with self._lock:
            run.exited = True
            run.exited_at = time.monotonic()
            if proc.run is run:
                proc.exit_code = code
                proc.prompt = None
                if not run.stop_requested:
                    proc.state = "exited"
                elif run.stop_done:
                    proc.state = "stopped"
            self._changed.notify_all()

    def _begin_stop_locked(self, proc: _Proc) -> None:
        """State stopping; the stop sequence runs on its own thread."""
        run = proc.run
        run.stop_requested = True
        proc.state = "stopping"
        proc.prompt = None
        threading.Thread(
            target=self._stop_sequence, args=(proc, run), name=f"stop-{proc.spec.name}", daemon=True
        ).start()

    def _stop_sequence(self, proc: _Proc, run: _Run) -> None:
        """Signals with their graces, then stop_argv; stopped at the end."""
        stop = proc.spec.stop
        with self._lock:
            for i, name in enumerate(stop.signals):
                if run.exited:
                    break
                self._signal_locked(run, name)
                grace = stop.grace_s[i] if i < len(stop.grace_s) else LAST_WAIT_S
                self._changed.wait_for(lambda: run.exited, grace)
        if run.stop_argv:
            self._run_stop_argv(proc, run)
        with self._lock:
            run.stop_done = True
            if run.exited and proc.run is run:
                proc.state = "stopped"
            self._changed.notify_all()

    def _signal_locked(self, run: _Run, name: str) -> None:
        """CTRL-C writes ^C to the pty; any other name goes to the process group."""
        if name == "CTRL-C":
            if run.tty and run.fd is not None:
                try:
                    os.write(run.fd, b"\x03")
                except OSError:
                    pass
            return
        try:
            os.killpg(run.popen.pid, getattr(signal, "SIG" + name))
        except OSError:
            pass  # the group is gone

    def _run_stop_argv(self, proc: _Proc, run: _Run) -> None:
        """Runs stop_argv with the start's cwd and env; its output goes to the log."""
        with self._lock:
            self._emit_locked(proc, [f"=== stop_argv: {shlex.join(run.stop_argv)} ==="])
        try:
            done = subprocess.run(
                run.stop_argv,
                cwd=run.cwd,
                env=run.env,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                timeout=STOP_ARGV_TIMEOUT_S,
                check=False,
            )
            output, result = done.stdout, f"exit code {done.returncode}"
        except subprocess.TimeoutExpired as exc:
            output, result = exc.output or b"", f"timed out after {STOP_ARGV_TIMEOUT_S:g} s"
        except (OSError, ValueError, subprocess.SubprocessError) as exc:
            output, result = b"", f"failed: {exc}"
        lines = output.decode("utf-8", "replace").replace("\r", "").split("\n")
        if lines[-1] == "":
            lines.pop()
        with self._lock:
            self._emit_locked(proc, [*lines, f"=== stop_argv {result} ==="])

    # --- readiness and conditions ---------------------------------------------

    def _watch(self) -> None:
        """Polls the http ready checks: starting -> ready, stopped/exited <-> external."""
        while not self._halt.wait(self._ready_period):
            for proc in self._procs.values():
                url = proc.spec.ready.http if proc.spec.ready else None
                if not url or self._halt.is_set():
                    continue
                with self._lock:
                    state, run = proc.state, proc.run
                    own = state in ("starting", "prompt") and run is not None and not run.ready
                if not own and state not in ("stopped", "exited", "external"):
                    continue
                ok = _http_ok(url)
                with self._lock:
                    if proc.state != state or proc.run is not run:
                        continue
                    if own:
                        if ok:
                            run.ready = True
                            if state == "starting":
                                proc.state = "ready"
                    elif ok != (state == "external"):
                        proc.state = "external" if ok else "stopped"

    def _conds(self) -> dict:
        """The external conditions now; a failing callback holds nothing."""
        if self._conditions is None:
            return {}
        try:
            return dict(self._conditions())
        except Exception:
            return {}

    def _requires_locked(self, spec: ProcessSpec, conds: Mapping) -> list[tuple[str, bool]]:
        """(cond, holds) for each of spec.requires."""
        result = []
        for cond in spec.requires:
            other, _, kind = cond.rpartition(".")
            if cond in conds:
                ok = bool(conds[cond])
            elif other in self._procs:
                ok = self._procs[other].state in _HOLDS.get(kind, ())
            else:
                ok = False
            result.append((cond, ok))
        return result

    def _describe_locked(self, proc: _Proc, conds: Mapping) -> dict:
        """The snapshot dict of one process."""
        spec, run = proc.spec, proc.run
        alive = proc.state in RUNNING and run is not None and not run.exited
        return {
            "name": spec.name,
            "label": spec.label or spec.name,
            "group": spec.group,
            "state": proc.state,
            "pid": run.popen.pid if alive else None,
            "exit_code": proc.exit_code,
            "ready": proc.state in _HOLDS["ready"],
            "external": proc.state == "external",
            "commands_robot": spec.commands_robot,
            "requires": [
                {"cond": cond, "ok": ok} for cond, ok in self._requires_locked(spec, conds)
            ],
            "params": {
                name: {
                    "choices": None if p.choices is None else list(p.choices),
                    "text": p.text,
                    "pattern": p.pattern,
                    "default": p.default,
                }
                for name, p in spec.params.items()
            },
            "prompt": None
            if proc.prompt is None
            else {"text": proc.prompt["text"], "answers": list(proc.prompt["answers"])},
            "log_seq": proc.seq,
            "log_path": proc.log_path,
            "partial": proc.partial,
            "started_at": proc.started_at,
        }


# --- helpers ------------------------------------------------------------------


def _param_values(spec: ProcessSpec, given: Mapping) -> dict[str, str]:
    """Every declared param: given, else its default; validated."""
    extra = sorted(set(given) - set(spec.params))
    if extra:
        raise _Refused(f"unknown param(s) {extra}")
    values = {}
    for name, param in spec.params.items():
        value = given.get(name, param.default)
        if value is None:
            raise _Refused(f"param {name}: missing")
        if not isinstance(value, str):
            raise _Refused(f"param {name}: expected a string")
        if any(c in value for c in "\0\n\r"):
            raise _Refused(f"param {name}: NUL or line break")
        if param.choices is not None and value not in param.choices:
            raise _Refused(f"param {name}: {value!r} is not one of {list(param.choices)}")
        if param.pattern is not None and not re.fullmatch(param.pattern, value):
            raise _Refused(f"param {name}: does not match {param.pattern}")
        values[name] = value
    return values


def _fill(element: str, spec: ProcessSpec, values: Mapping[str, str]) -> str:
    """Replaces declared "{param}" placeholders; a text param only as the whole element."""

    def value(match):
        name = match.group(1)
        if name not in spec.params:
            return match.group(0)
        if spec.params[name].text and match.group(0) != element:
            raise _Refused(f"text param {name} must be a whole argv element")
        return values[name]

    return PLACEHOLDER.sub(value, element)


def _check_program(program: str, cwd: str | None, env: Mapping[str, str]) -> None:
    """Raises OSError unless exec finds `program`; under setpriv Popen cannot tell."""
    if "/" in program:
        path = os.path.join(cwd or os.getcwd(), program)
        if not os.path.isfile(path):
            raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), program)
        if not os.access(path, os.X_OK):
            raise PermissionError(errno.EACCES, os.strerror(errno.EACCES), program)
    elif shutil.which(program, path=env.get("PATH", os.defpath)) is None:
        raise FileNotFoundError(errno.ENOENT, "not found on PATH", program)


def _spawn(command: list[str], cwd: str | None, env: Mapping[str, str], tty: bool):
    """Popen in a new session, output to a pipe or a pty; returns (popen, parent fd)."""
    parent, child = pty.openpty() if tty else os.pipe()
    try:
        popen = subprocess.Popen(
            command,
            cwd=cwd,
            env=env,
            stdin=child if tty else subprocess.DEVNULL,
            stdout=child,
            stderr=child,
            start_new_session=True,
            close_fds=True,
        )
    except BaseException:
        os.close(parent)
        raise
    finally:
        os.close(child)
    os.set_blocking(parent, False)
    return popen, parent


def _http_ok(url: str) -> bool:
    """True when `url` answers 200 within 1 s; proxies ignored."""
    try:
        with _NO_PROXY.open(url, timeout=1.0) as response:
            return response.status == 200
    except Exception:
        return False
