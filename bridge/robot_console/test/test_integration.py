"""The real server, service and page against devserver's simulated bridge and robot.

One operator's session end to end over HTTP and the WebSocket: take control,
arm, jog a joint, e-stop, see the refusal, reset, re-arm. No ROS, no browser.
"""

from __future__ import annotations

import base64
import http.client
import json
import os
import socket
import threading
import time

import pytest
from robot_console import auth
from robot_console import server as server_mod
from robot_console.devserver import FakeBridge, FakeRobot
from robot_console.service import ConsoleService
from robot_console.session import ProbeSession

TOKEN = "integration-token"


@pytest.fixture
def stack(catalog, parts):
    config = parts[0]._config
    bridge = FakeBridge()
    service = ConsoleService(config, catalog, ProbeSession(catalog), parts[1], bridge.estop)
    parts[3].close()  # the FakeRobot below is this stack's loop
    robot = FakeRobot(service, catalog, bridge)
    stop = threading.Event()

    def loop():
        last = time.monotonic()
        while not stop.wait(0.01):
            now = time.monotonic()
            robot.step(now - last)
            last = now

    threading.Thread(target=loop, daemon=True).start()
    httpd = server_mod.serve(service, TOKEN, port=0)
    threading.Thread(
        target=httpd.serve_forever, kwargs={"poll_interval": 0.02}, daemon=True
    ).start()
    yield httpd.server_address, service, bridge
    stop.set()
    httpd.shutdown()
    httpd.server_close()


class Tab:
    """One browser tab: a stream socket plus POSTs carrying its client id."""

    def __init__(self, address):
        self.address = address
        self.sock = socket.create_connection(address, timeout=5)
        key = base64.b64encode(os.urandom(16)).decode()
        self.sock.sendall(
            (
                "GET /api/stream HTTP/1.1\r\n"
                f"Host: {address[0]}:{address[1]}\r\n"
                f"Cookie: {auth.COOKIE}={TOKEN}\r\n"
                "Upgrade: websocket\r\nConnection: Upgrade\r\n"
                f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n"
            ).encode()
        )
        header = b""
        while b"\r\n\r\n" not in header:
            header += self.sock.recv(1)
        assert b"101" in header.split(b"\r\n")[0]
        self.stream = self.sock.makefile("rb")
        self.client = self.next()["client"]
        self.last = None
        self._stop = threading.Event()
        threading.Thread(target=self._pump, daemon=True).start()

    def next(self):
        opcode, payload = server_mod.read_message(self.stream)
        return json.loads(payload)

    def _pump(self):
        mask = b"\x01\x02\x03\x04"
        ping = bytes(b ^ mask[i % 4] for i, b in enumerate(b'{"type":"ping"}'))
        last_ping = 0.0
        while not self._stop.is_set():
            try:
                self.last = self.next()
                if time.monotonic() - last_ping > 0.2:
                    self.sock.sendall(bytes([0x81, 0x80 | len(ping)]) + mask + ping)
                    last_ping = time.monotonic()
            except (OSError, ValueError):
                return

    def post(self, path, **body):
        conn = http.client.HTTPConnection(*self.address, timeout=15)
        conn.request(
            "POST",
            path,
            body=json.dumps({**body, "client": self.client}),
            headers={"Cookie": f"{auth.COOKIE}={TOKEN}", "Content-Type": "application/json"},
        )
        reply = conn.getresponse()
        data = json.loads(reply.read())
        conn.close()
        return reply.status, data

    def wait(self, cond, timeout=5.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.last is not None and cond(self.last):
                return self.last
            time.sleep(0.02)
        raise AssertionError(f"timed out; last snapshot bridge={self.last and self.last['bridge']}")

    def close(self):
        self._stop.set()
        self.stream.close()
        self.sock.close()


def test_one_operator_session(stack):
    address, service, bridge = stack
    tab = Tab(address)
    try:
        assert tab.post("/api/arm", confirm=True)[0] == 409, "no control yet"
        assert tab.post("/api/control/take")[1]["ok"] is True
        assert tab.post("/api/arm", confirm=True)[1]["ok"] is True
        tab.wait(lambda s: s["bridge"]["state"] == "ARMED")

        assert tab.post("/api/jog/start")[1]["ok"] is True
        tab.wait(lambda s: s["jog"]["active"] and s["jog"]["alive"])
        assert tab.post("/api/jog/target", name="L_ELBOW", q=0.5)[1]["q"] == pytest.approx(0.5)
        elbow = lambda s: next(m for m in s["motors"] if m["name"] == "L_ELBOW")  # noqa: E731
        tab.wait(lambda s: abs((elbow(s)["q_meas"] or 0) - 0.5) < 0.05)

        assert tab.post("/api/estop", mode="damp")[1]["ok"] is True
        tab.wait(
            lambda s: s["bridge"]["state"] == "LATCHED damp service_damp" and not s["jog"]["active"]
        )
        status, reply = tab.post("/api/arm", confirm=True)
        assert reply["ok"] is False and "estop_reset" in reply["message"]

        assert tab.post("/api/reset")[1]["ok"] is True
        tab.wait(lambda s: s["bridge"]["state"] == "STOPPED damp service_damp")
        assert tab.post("/api/arm", confirm=True)[1]["ok"] is True
        tab.wait(lambda s: s["bridge"]["state"] == "ARMED")
    finally:
        tab.close()


def test_a_second_tab_can_estop_but_not_arm(stack):
    address, *_ = stack
    owner, viewer = Tab(address), Tab(address)
    try:
        owner.post("/api/control/take")
        viewer.wait(lambda s: s["control"]["held"] and not s["control"]["yours"])
        assert viewer.post("/api/arm", confirm=True)[0] == 409
        assert viewer.post("/api/estop", mode="hold")[1]["ok"] is True
        owner.wait(lambda s: s["bridge"]["state"].startswith("LATCHED hold"))
    finally:
        owner.close()
        viewer.close()


def test_the_page_carries_the_safety_bar(stack):
    address, *_ = stack
    conn = http.client.HTTPConnection(*address, timeout=5)
    conn.request("GET", "/", headers={"Cookie": f"{auth.COOKIE}={TOKEN}"})
    body = conn.getresponse().read()
    for marker in (
        b'id="btn-estop-damp"',
        b'id="btn-estop-hold"',
        b'id="btn-reset"',
        b"console.js",
    ):
        assert marker in body
