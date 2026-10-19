"""Supervisor against real child processes."""

from __future__ import annotations

import ast
import datetime
import http.server
import os
import re
import shutil
import signal
import socket
import sys
import threading
import time
from pathlib import Path

import pytest
from robot_console.config import ParamSpec, ProcessSpec, PromptSpec, ReadySpec, StopSpec
from robot_console.supervisor import Supervisor

PY = sys.executable
# trap: a SIGINT during python startup can be lost: stop a child only after its "up"
SLEEP = "import time; print('up'); time.sleep(30)"
PROMPT = PromptSpec(r"ready\? \[go/abort\]", ("go", "abort"))
EXTERNAL = (False, "external: started outside the console")
KEYS = {
    "name", "label", "group", "state", "pid", "exit_code", "ready", "external", "commands_robot",
    "requires", "params", "prompt", "log_seq", "log_path", "partial", "started_at",
}  # fmt: skip


def py(name, script, **kw):
    """A spec that runs `script` in an unbuffered python."""
    return ProcessSpec(name=name, argv=(PY, "-u", "-c", script), **kw)


def wait_until(cond, timeout=5.0):
    """Polls `cond` every 20 ms; its last value after `timeout`."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.02)
    return cond()


def info(sup, name):
    return next(p for p in sup.snapshot() if p["name"] == name)


def state(sup, name):
    return info(sup, name)["state"]


def lines(sup, name):
    return sup.log(name)["lines"]


def free_port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


class _Ok(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.end_headers()

    def log_message(self, *args):
        pass


@pytest.fixture
def make(tmp_path):
    made = []

    def make(*specs, **kw):
        kw.setdefault("ready_period", 0.1)
        sup = Supervisor(specs, tmp_path / "logs", **kw)
        made.append(sup)
        return sup

    yield make
    for sup in made:
        sup.shutdown(timeout=5)


# --- start, output, exit ------------------------------------------------------


def test_start_logs_lines_seq_and_exit_code(make, tmp_path):
    sup = make(py("p", "print('one'); print('two'); raise SystemExit(3)"))
    ok, reason = sup.start("p")
    assert ok, reason
    assert wait_until(lambda: state(sup, "p") == "exited" and sup.log("p")["seq"] == 3)
    snap = info(sup, "p")
    assert snap["exit_code"] == 3 and snap["pid"] is None and snap["log_seq"] == 3
    log = sup.log("p")
    assert log["lines"][0].startswith("=== start ") and log["lines"][0].endswith(" ===")
    assert log["lines"][1:] == ["one", "two"]
    assert sup.log("p", since=2) == {"seq": 3, "lines": ["two"]}
    assert sup.log("p", since=3)["lines"] == []

    path = Path(snap["log_path"])
    assert path.parent == tmp_path / "logs" / datetime.date.today().isoformat()
    assert re.fullmatch(r"p-\d{6}\.log", path.name)
    assert path.read_text().splitlines() == log["lines"]

    assert sup.start("p")[0]
    assert wait_until(lambda: state(sup, "p") == "exited" and sup.log("p")["seq"] == 6)
    again = sup.log("p", since=3)["lines"]
    assert again[0].startswith("=== start ") and again[1:] == ["one", "two"]


def test_ring_keeps_the_last_200_lines(make):
    sup = make(py("p", "for i in range(250): print(i)"))
    assert sup.start("p")[0]
    assert wait_until(lambda: sup.log("p")["seq"] == 251)
    got = lines(sup, "p")
    assert len(got) == 200 and got[0] == "50" and got[-1] == "249"


def test_unfinished_line_is_partial_and_log_ready_matches_it(make):
    script = (
        "import sys, time\n"
        "print('warming')\n"
        "time.sleep(0.3)\n"
        "sys.stdout.write('listening on :80')\n"
        "time.sleep(30)\n"
    )
    sup = make(py("p", script, ready=ReadySpec(log=r"listening on :\d+")))
    assert sup.start("p")[0]
    assert state(sup, "p") == "starting"
    assert wait_until(lambda: "warming" in lines(sup, "p"))
    assert state(sup, "p") == "starting"
    assert wait_until(lambda: state(sup, "p") == "ready")
    snap = info(sup, "p")
    assert snap["partial"] == "listening on :80" and snap["ready"]
    assert "listening on :80" not in lines(sup, "p")


# --- stop -----------------------------------------------------------------------


def test_stop_with_int_handled_by_the_child(make):
    script = (
        "import signal, sys, time\n"
        "def bye(*_):\n"
        "    print('bye')\n"
        "    sys.exit(0)\n"
        "signal.signal(signal.SIGINT, bye)\n"
        "print('up')\n"
        "time.sleep(30)\n"
    )
    sup = make(py("p", script, stop=StopSpec(("INT", "TERM", "KILL"), (5.0, 5.0))))
    assert sup.start("p")[0]
    assert wait_until(lambda: "up" in lines(sup, "p"))
    assert sup.stop("p") == (True, "stopping")
    assert wait_until(lambda: state(sup, "p") == "stopped")
    assert info(sup, "p")["exit_code"] == 0
    assert wait_until(lambda: "bye" in lines(sup, "p"))
    assert sup.stop("p") == (False, "not running")


def test_stop_escalates_to_kill(make):
    script = (
        "import signal, time\n"
        "signal.signal(signal.SIGINT, signal.SIG_IGN)\n"
        "signal.signal(signal.SIGTERM, signal.SIG_IGN)\n"
        "print('up')\n"
        "time.sleep(30)\n"
    )
    sup = make(py("p", script, stop=StopSpec(("INT", "TERM", "KILL"), (0.2, 0.2))))
    assert sup.start("p")[0]
    assert wait_until(lambda: "up" in lines(sup, "p"))
    assert sup.stop("p")[0]
    assert sup.stop("p") == (False, "already stopping")
    assert wait_until(lambda: state(sup, "p") == "stopped")
    assert info(sup, "p")["exit_code"] == -signal.SIGKILL


def test_stop_argv_runs_with_the_start_env_and_cwd(make, tmp_path):
    clean = "import os; print('cleaned', os.environ['MARK'], os.getcwd())"
    spec = py(
        "p",
        SLEEP,
        cwd=str(tmp_path),
        env={"MARK": "m1"},
        stop=StopSpec(("TERM",), ()),
        stop_argv=(PY, "-c", clean),
    )
    sup = make(spec)
    assert sup.start("p")[0]
    assert wait_until(lambda: "up" in lines(sup, "p"))
    assert sup.stop("p")[0]
    assert wait_until(lambda: state(sup, "p") == "stopped")
    got = lines(sup, "p")
    assert f"cleaned m1 {os.path.realpath(tmp_path)}" in got
    assert got[-1] == "=== stop_argv exit code 0 ==="
    assert info(sup, "p")["exit_code"] == -signal.SIGTERM


# --- refusals -------------------------------------------------------------------


def test_start_refusals(make, tmp_path):
    params = {
        "ckpt": ParamSpec("ckpt", choices=("a", "b")),
        "msg": ParamSpec("msg", text=True, pattern=r"[a-z ]*"),
        "free": ParamSpec("free", text=True),
    }
    sup = make(
        ProcessSpec("p", (PY, "-u", "-c", SLEEP, "{ckpt}", "{msg}", "{free}"),
                    params=params),
        ProcessSpec("nocwd", (PY, "-c", "pass"), cwd=str(tmp_path / "missing")),
        ProcessSpec("noexec", (str(tmp_path / "missing-binary"),)),
    )  # fmt: skip
    good = {"ckpt": "a", "msg": "hello", "free": "x"}
    assert sup.start("nope") == (False, "unknown process 'nope'")
    for bad, word in [
        ({**good, "ckpt": "c"}, "not one of"),
        ({**good, "msg": "Hello!"}, "does not match"),
        ({**good, "free": "two\nlines"}, "line break"),
        ({**good, "free": "nul\0"}, "line break"),
        ({"ckpt": "a", "free": "x"}, "msg: missing"),
        ({**good, "extra": "x"}, "unknown param"),
    ]:
        ok, reason = sup.start("p", bad)
        assert not ok and word in reason, (bad, reason)
    assert state(sup, "p") == "stopped" and lines(sup, "p") == []

    ok, reason = sup.start("nocwd")
    assert not ok and reason.startswith("cwd not found")
    ok, reason = sup.start("noexec")
    assert not ok and "exec failed" in reason and "missing-binary" in reason
    assert state(sup, "noexec") == "stopped"
    assert any("exec failed" in line for line in lines(sup, "noexec"))

    assert sup.start("p", good)[0]
    assert sup.start("p", good) == (False, "already ready")
    assert wait_until(lambda: "up" in lines(sup, "p"))


def test_exec_failure_from_popen_without_setpriv(make, tmp_path):
    script = tmp_path / "bad.sh"
    script.write_text("#!/nonexistent/interpreter\n")
    script.chmod(0o755)
    sup = make(ProcessSpec("p", (str(script),)))
    sup._setpriv = None  # Popen itself execs the program
    ok, reason = sup.start("p")
    assert not ok and "exec failed" in reason
    assert state(sup, "p") == "stopped"
    got = lines(sup, "p")
    assert any("setpriv not found" in line for line in got)
    assert any("exec failed" in line for line in got)


# --- params and environment -------------------------------------------------------


def test_params_fill_argv_and_are_never_env_expanded(make):
    params = {"ckpt": ParamSpec("ckpt", choices=("task3",)), "say": ParamSpec("say", text=True)}
    argv = (PY, "-c", "import sys; print(sys.argv[1:])", "--ckpt=run_{ckpt}_x", "{say}", "${HOME}")
    sup = make(ProcessSpec("p", argv, params=params), environ={"HOME": "/home/x"})
    value = "hi ${HOME} $HOME ~ {ckpt}"
    assert sup.start("p", {"ckpt": "task3", "say": value})[0]
    assert wait_until(lambda: state(sup, "p") == "exited" and len(lines(sup, "p")) == 2)
    assert ast.literal_eval(lines(sup, "p")[-1]) == ["--ckpt=run_task3_x", value, "/home/x"]


def test_text_param_inside_an_element_is_refused(make):
    params = {"say": ParamSpec("say", text=True)}
    sup = make(ProcessSpec("p", (PY, "-c", "print('{say}')"), params=params))
    ok, reason = sup.start("p", {"say": "x"})
    assert not ok and "whole argv element" in reason


def test_environment_precedence(make, tmp_path):
    work = tmp_path / "work"
    work.mkdir()
    (work / ".env").write_text("A=file\nB=file\nC=file\n")
    base = {"PATH": os.environ.get("PATH", ""), "A": "base", "WORK": str(work), "ROOT": "/r"}
    script = (
        "import os, sys; "
        "print(repr(([os.environ.get(k) for k in 'ABCDEF'], sys.argv[1:], os.getcwd())))"
    )
    spec = ProcessSpec(
        "p",
        (PY, "-c", script, "${C}", "$ROOT/x"),
        cwd="${WORK}",
        env_file=".env",
        env={"B": "spec", "D": "${A}-${C}", "E": "${ROOT}/sub", "F": "${B}!"},
    )
    sup = make(spec, environ=base)
    assert sup.start("p")[0]
    assert wait_until(lambda: state(sup, "p") == "exited" and len(lines(sup, "p")) == 2)
    values, args, cwd = ast.literal_eval(lines(sup, "p")[-1])
    assert values == ["base", "spec", "file", "base-file", "/r/sub", "spec!"]
    assert args == ["file", "/r/x"]
    assert cwd == os.path.realpath(work)


def test_missing_env_file_is_refused(make, tmp_path):
    sup = make(ProcessSpec("p", (PY, "-c", "pass"), cwd=str(tmp_path), env_file="nope.env"))
    ok, reason = sup.start("p")
    assert not ok and reason.startswith("env_file")


# --- requires --------------------------------------------------------------------


def test_requires_armed_ready_and_running(make):
    conds = {"armed": False}
    dep = "import time; time.sleep(0.3); print('dep ready'); time.sleep(30)"
    sup = make(
        py("dep", dep, ready=ReadySpec(log="^dep ready$")),
        py("other", SLEEP),
        py("needs", SLEEP, requires=("armed", "dep.ready", "other.running")),
        py("watch", SLEEP, requires=("dep.running",)),
        conditions=lambda: conds,
    )
    assert sup.requires("needs") == [
        ("armed", False),
        ("dep.ready", False),
        ("other.running", False),
    ]
    assert sup.start("needs") == (False, "requires armed")
    conds["armed"] = True
    assert sup.start("needs") == (False, "requires dep.ready")

    assert sup.start("dep")[0]
    assert sup.requires("watch") == [("dep.running", True)]
    assert sup.requires("needs")[1] == ("dep.ready", False)
    assert wait_until(lambda: sup.requires("needs")[1] == ("dep.ready", True))
    assert sup.start("needs") == (False, "requires other.running")

    assert sup.start("other")[0]
    assert info(sup, "needs")["requires"] == [
        {"cond": "armed", "ok": True},
        {"cond": "dep.ready", "ok": True},
        {"cond": "other.running", "ok": True},
    ]
    assert sup.start("needs")[0]
    assert wait_until(lambda: "up" in lines(sup, "needs") and "up" in lines(sup, "other"))


# --- http readiness and external ------------------------------------------------------


def test_http_ready_moves_starting_to_ready(make):
    server = (
        "import http.server, sys, time\n"
        "class H(http.server.BaseHTTPRequestHandler):\n"
        "    def do_GET(self):\n"
        "        self.send_response(200)\n"
        "        self.end_headers()\n"
        "    def log_message(self, *args):\n"
        "        pass\n"
        "time.sleep(0.3)\n"
        "http.server.HTTPServer(('127.0.0.1', int(sys.argv[1])), H).serve_forever()\n"
    )
    port = free_port()
    url = f"http://127.0.0.1:{port}/health"
    sup = make(ProcessSpec("srv", (PY, "-c", server, str(port)), ready=ReadySpec(http=url)))
    assert sup.start("srv")[0]
    assert state(sup, "srv") == "starting"
    assert wait_until(lambda: state(sup, "srv") == "ready")
    assert sup.stop("srv")[0]
    assert wait_until(lambda: state(sup, "srv") == "stopped")
    time.sleep(0.3)
    assert state(sup, "srv") == "stopped"


def test_server_started_outside_shows_external(make):
    server = http.server.HTTPServer(("127.0.0.1", 0), _Ok)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    url = f"http://127.0.0.1:{server.server_address[1]}/"
    try:
        sup = make(py("ext", SLEEP, ready=ReadySpec(http=url)))
        assert wait_until(lambda: state(sup, "ext") == "external")
        snap = info(sup, "ext")
        assert snap["external"] and snap["ready"] and snap["pid"] is None
        assert sup.start("ext") == EXTERNAL
        assert sup.stop("ext") == EXTERNAL
    finally:
        server.shutdown()
        server.server_close()
    assert wait_until(lambda: state(sup, "ext") == "stopped")


# --- tty and prompts -------------------------------------------------------------


def test_tty_prompt_is_answered(make):
    script = "print('hello')\nanswer = input('ready? [go/abort] ')\nprint('got ' + answer)\n"
    sup = make(py("p", script, tty=True, prompts=(PROMPT,)))
    assert sup.answer("p", "go") == (False, "no prompt pending")
    assert sup.start("p")[0]
    assert wait_until(lambda: state(sup, "p") == "prompt")
    prompt = info(sup, "p")["prompt"]
    assert "ready? [go/abort]" in prompt["text"] and prompt["answers"] == ["go", "abort"]
    ok, reason = sup.answer("p", "maybe")
    assert not ok and "one of" in reason
    assert sup.answer("p", "go") == (True, "answered")
    assert info(sup, "p")["prompt"] is None
    assert wait_until(lambda: "got go" in lines(sup, "p"))
    assert wait_until(lambda: state(sup, "p") == "exited")
    assert info(sup, "p")["exit_code"] == 0


def test_non_tty_spec_has_no_prompts(make):
    script = "import time; print('ready? [go/abort] '); time.sleep(30)"
    sup = make(py("p", script, prompts=(PROMPT,)))
    assert sup.start("p")[0]
    assert wait_until(lambda: "ready? [go/abort] " in lines(sup, "p"))
    snap = info(sup, "p")
    assert snap["state"] == "ready" and snap["prompt"] is None
    assert sup.answer("p", "go")[0] is False


def test_ctrl_c_stops_a_tty_child(make):
    script = (
        "import time\n"
        "print('up')\n"
        "try:\n"
        "    time.sleep(30)\n"
        "except KeyboardInterrupt:\n"
        "    print('interrupted')\n"
    )
    sup = make(py("p", script, tty=True, stop=StopSpec(("CTRL-C", "KILL"), (5.0,))))
    assert sup.start("p")[0]
    assert wait_until(lambda: "up" in lines(sup, "p"))
    assert sup.stop("p")[0]
    assert wait_until(lambda: state(sup, "p") == "stopped")
    assert info(sup, "p")["exit_code"] == 0
    # the pty echoes the ^C on the same line
    assert wait_until(lambda: "^Cinterrupted" in lines(sup, "p"))


# --- process lifetime ----------------------------------------------------------------


@pytest.mark.skipif(shutil.which("setpriv") is None, reason="needs setpriv")
def test_child_outlives_the_thread_that_called_start(make):
    sup = make(py("p", SLEEP))
    result = []

    def call():
        result.append(sup.start("p"))
        time.sleep(0.2)  # exit after setpriv armed the death signal

    caller = threading.Thread(target=call)
    caller.start()
    caller.join()
    assert result[0][0], result
    time.sleep(0.5)
    snap = info(sup, "p")
    assert snap["state"] == "ready"
    os.kill(snap["pid"], 0)


def test_snapshot_and_running_commands_robot(make):
    params = {"ckpt": ParamSpec("ckpt", choices=("a", "b"), default="a")}
    robot = ProcessSpec(
        "robot",
        (PY, "-u", "-c", SLEEP, "{ckpt}"),
        label="Robot",
        group="policy",
        params=params,
        commands_robot=True,
        stop=StopSpec(("TERM",), ()),
    )
    sup = make(robot, py("cam", SLEEP))
    assert sup.running_commands_robot() == []
    fresh = info(sup, "robot")
    assert fresh["pid"] is None and fresh["log_path"] is None and fresh["started_at"] is None
    assert sup.start("robot")[0] and sup.start("cam")[0]
    assert wait_until(lambda: "up" in lines(sup, "robot") and "up" in lines(sup, "cam"))
    assert sup.running_commands_robot() == ["robot"]

    snaps = sup.snapshot()
    assert [s["name"] for s in snaps] == ["robot", "cam"]
    assert all(set(s) == KEYS for s in snaps)
    snap = snaps[0]
    assert (snap["label"], snap["group"], snap["state"]) == ("Robot", "policy", "ready")
    assert snap["commands_robot"] and snap["pid"] > 0 and snap["exit_code"] is None
    assert snap["started_at"] and snap["log_path"] and snap["prompt"] is None
    assert snap["params"] == {
        "ckpt": {"choices": ["a", "b"], "text": False, "pattern": None, "default": "a"}
    }
    assert sup.stop("robot")[0]
    assert wait_until(lambda: state(sup, "robot") == "stopped")
    assert sup.running_commands_robot() == []


def test_shutdown_stops_everything_and_is_idempotent(tmp_path):
    sup = Supervisor([py("a", SLEEP), py("b", SLEEP)], tmp_path / "logs", ready_period=0.1)
    assert sup.start("a")[0] and sup.start("b")[0]
    assert wait_until(lambda: "up" in lines(sup, "a") and "up" in lines(sup, "b"))
    pids = [s["pid"] for s in sup.snapshot()]
    sup.shutdown(timeout=10)
    assert [s["state"] for s in sup.snapshot()] == ["stopped", "stopped"]
    for pid in pids:
        with pytest.raises(ProcessLookupError):
            os.kill(pid, 0)
    begin = time.monotonic()
    sup.shutdown(timeout=10)
    assert time.monotonic() - begin < 1.0
    assert sup.start("a") == (False, "shutting down")
