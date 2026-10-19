"""The HTTP + WebSocket surface: auth, routes, refusals, the camera proxy, the stream."""

from __future__ import annotations

import base64
import http.client
import json
import os
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest
from robot_console import auth
from robot_console import server as server_mod

TOKEN = "test-token"
COOKIE = f"{auth.COOKIE}={TOKEN}"


def start(httpd):
    thread = threading.Thread(
        target=httpd.serve_forever, kwargs={"poll_interval": 0.02}, daemon=True
    )
    thread.start()
    return thread


@pytest.fixture
def camera():
    """A stand-in camera preview serving one fixed JPEG-ish body."""

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            body = b"\xff\xd8fakejpeg\xff\xd9"
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    httpd = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    start(httpd)
    yield f"http://127.0.0.1:{httpd.server_address[1]}/frame.jpg"
    httpd.shutdown()
    httpd.server_close()


@pytest.fixture
def live(parts, tmp_path, camera):
    service = parts[0]
    assets = tmp_path / "assets"
    (assets / "meshes").mkdir(parents=True)
    (assets / "robot.urdf").write_text("<robot/>")
    vendor = tmp_path / "vendor" / "three"
    vendor.mkdir(parents=True)
    (vendor / "three.module.js").write_text("export const x = 1;")
    httpd = server_mod.serve(
        service,
        TOKEN,
        port=0,
        assets_root=assets,
        vendor_root=tmp_path / "vendor",
        camera_url=camera,
    )
    start(httpd)
    yield httpd.server_address, parts
    httpd.shutdown()
    httpd.server_close()


def request(address, method, path, body=None, cookie=True, headers=None):
    conn = http.client.HTTPConnection(*address, timeout=5)
    head = {"Host": f"{address[0]}:{address[1]}"}
    if cookie:
        head["Cookie"] = COOKIE
    if body is not None:
        head["Content-Type"] = "application/json"
    head.update(headers or {})
    conn.request(method, path, body=None if body is None else json.dumps(body), headers=head)
    reply = conn.getresponse()
    data = reply.read()
    conn.close()
    return reply.status, dict(reply.getheaders()), data


def post(address, path, body=None, **kw):
    status, _, data = request(address, "POST", path, body or {}, **kw)
    return status, json.loads(data)


# --- auth ---------------------------------------------------------------


def test_no_cookie_no_page(live):
    address, _ = live
    status, _, body = request(address, "GET", "/", cookie=False)
    assert status == 401 and b"token" in body
    status, _, _ = request(address, "GET", "/api/catalog", cookie=False)
    assert status == 401
    status, _ = post(address, "/api/estop", {"mode": "damp"}, cookie=False)
    assert status == 401


def test_url_token_becomes_a_cookie(live):
    address, _ = live
    status, headers, _ = request(address, "GET", f"/?token={TOKEN}", cookie=False)
    assert status == 303
    assert headers["Location"] == "/"
    assert headers["Set-Cookie"].startswith(f"{auth.COOKIE}={TOKEN}")
    status, _, _ = request(address, "GET", "/?token=wrong", cookie=False)
    assert status == 401


def test_cross_origin_post_is_refused(live):
    address, (_, _, estops, _) = live
    status, reply = post(
        address, "/api/estop", {"mode": "damp"}, headers={"Origin": "http://evil.example"}
    )
    assert status == 403 and estops == []


# --- pages and files ------------------------------------------------------


def test_page_and_modules_are_served(live):
    address, _ = live
    status, headers, body = request(address, "GET", "/")
    assert status == 200 and b"E-STOP DAMP" in body
    for path in ("/console.js", "/jog.js"):
        status, headers, _ = request(address, "GET", path)
        assert status == 200 and headers["Content-Type"] == "text/javascript"


def test_assets_and_vendor_roots(live):
    address, _ = live
    assert request(address, "GET", "/assets/robot.urdf")[0] == 200
    assert request(address, "GET", "/vendor/three/three.module.js")[0] == 200
    assert request(address, "GET", "/assets/../../../../etc/passwd")[0] == 404
    assert request(address, "GET", "/vendor/../assets/robot.urdf")[0] == 404


def test_catalog(live):
    address, _ = live
    status, _, body = request(address, "GET", "/api/catalog")
    assert status == 200 and len(json.loads(body)["motors"]) == 43


def test_camera_is_proxied(live):
    address, _ = live
    status, headers, body = request(address, "GET", "/api/camera/frame.jpg")
    assert (
        status == 200 and headers["Content-Type"] == "image/jpeg" and body.startswith(b"\xff\xd8")
    )


def test_camera_failure_is_a_502(parts):
    httpd = server_mod.serve(parts[0], TOKEN, port=0, camera_url="http://127.0.0.1:9/frame.jpg")
    start(httpd)
    try:
        status, _, _ = request(httpd.server_address, "GET", "/api/camera/frame.jpg")
        assert status == 502
    finally:
        httpd.shutdown()
        httpd.server_close()


def test_no_camera_is_a_404(parts):
    httpd = server_mod.serve(parts[0], TOKEN, port=0)
    start(httpd)
    try:
        assert request(httpd.server_address, "GET", "/api/camera/frame.jpg")[0] == 404
    finally:
        httpd.shutdown()
        httpd.server_close()


def test_process_log(live):
    address, _ = live
    status, _, body = request(address, "GET", "/api/process/log?name=camera&since=1")
    assert status == 200 and json.loads(body) == {"seq": 3, "lines": ["b", "c"]}
    assert request(address, "GET", "/api/process/log?name=nope")[0] == 400


# --- routes and refusals -----------------------------------------------------


def test_estop_any_tab(live):
    address, (_, _, estops, _) = live
    status, reply = post(address, "/api/estop", {"mode": "hold"})
    assert status == 200 and reply["ok"] is True and estops == ["hold"]
    assert post(address, "/api/estop", {"mode": "limp"})[0] == 400


def test_refusal_is_409_with_the_reason(live):
    address, _ = live
    status, reply = post(address, "/api/arm", {"confirm": True, "client": "nobody"})
    assert status == 409 and "control" in reply["message"]


def test_bad_requests(live):
    address, _ = live
    assert post(address, "/api/nope")[0] == 404
    assert post(address, "/api/process/stop", {})[0] == 404  # missing field
    conn = http.client.HTTPConnection(*address, timeout=5)
    conn.request("POST", "/api/stop", body=b"not json", headers={"Cookie": COOKIE})
    assert conn.getresponse().status == 400
    conn.close()


# --- the stream ---------------------------------------------------------------


def ws_connect(address, origin=None):
    sock = socket.create_connection(address, timeout=5)
    key = base64.b64encode(os.urandom(16)).decode()
    lines = [
        "GET /api/stream HTTP/1.1",
        f"Host: {address[0]}:{address[1]}",
        f"Cookie: {COOKIE}",
        "Upgrade: websocket",
        "Connection: Upgrade",
        f"Sec-WebSocket-Key: {key}",
        "Sec-WebSocket-Version: 13",
    ]
    if origin:
        lines.append(f"Origin: {origin}")
    sock.sendall(("\r\n".join(lines) + "\r\n\r\n").encode())
    header = b""
    while b"\r\n\r\n" not in header:
        chunk = sock.recv(1)
        if not chunk:
            break
        header += chunk
    return sock, header


def client_frame(payload):
    mask = os.urandom(4)
    masked = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
    return bytes([0x81, 0x80 | len(payload)]) + mask + masked


def next_message(stream):
    opcode, payload = server_mod.read_message(stream)
    assert opcode == 0x1
    return json.loads(payload)


def wait_until(cond, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.01)
    return False


def test_stream_hello_snapshot_ping_and_disconnect(live):
    address, (service, *_) = live
    sock, header = ws_connect(address, origin=f"http://{address[0]}:{address[1]}")
    assert b"101" in header.split(b"\r\n")[0]
    stream = sock.makefile("rb")
    try:
        hello = next_message(stream)
        assert hello["type"] == "hello"
        client = hello["client"]
        service.take_control(client)
        snap = next_message(stream)
        assert snap["type"] == "state" and snap["control"]["yours"] is True

        service.set_bridge_state("ARMED")
        service.set_jog_active(True)
        sock.sendall(client_frame(b'{"type":"ping"}'))
        assert wait_until(lambda: service.jog_commands()[0]), (
            "the controller's ping arms the deadman"
        )
    finally:
        stream.close()  # makefile() holds the fd open: close both, or the server never sees EOF
        sock.close()
    assert wait_until(lambda: not service.jog_commands()[0]), "a closed socket trips the deadman"
    assert wait_until(lambda: not service.snapshot()["control"]["held"]), "and frees the lock"


def test_cross_origin_stream_is_refused(live):
    address, _ = live
    sock, header = ws_connect(address, origin="http://evil.example")
    sock.close()
    assert b"403" in header.split(b"\r\n")[0]
