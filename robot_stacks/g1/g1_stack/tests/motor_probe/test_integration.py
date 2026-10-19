"""The real server, the real catalog and the real page, wired together.

Covers the seam the unit tests miss: `server.py` serving the actual `static/`
tree and the fetched assets, and a real WebSocket session doing the ping ->
snapshot exchange the deadman depends on. No ROS, no browser.
"""

import base64
import json
import os
import socket
import threading
import urllib.error
import urllib.request

import pytest

from g1_stack.motor_probe import server as server_mod
from g1_stack.motor_probe.catalog import load_catalog
from g1_stack.motor_probe.session import ProbeSession
from g1_stack.nodes.motor_probe import ProbeService

ASSETS = server_mod.STATIC_ROOT.parents[2] / "assets"


@pytest.fixture
def live():
    catalog = load_catalog()
    service = ProbeService(catalog, ProbeSession(catalog))
    assets = ASSETS if ASSETS.is_dir() else None
    httpd = server_mod.serve(service, host="127.0.0.1", port=0, assets_root=assets)
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    host, port = httpd.server_address[0], httpd.server_address[1]
    yield f"http://{host}:{port}", (host, port), service
    httpd.shutdown()
    httpd.server_close()
    thread.join(timeout=2.0)


def fetch(url):
    with urllib.request.urlopen(url, timeout=5) as response:
        return response.status, response.read()


# --- the page ------------------------------------------------------------


def test_every_default_assets_path_agrees():
    """Three files derive this path independently; they must land in one place.

    They did not, once: the node and the devserver were each one directory too
    shallow, so `--assets` silently defaulted to a directory that never exists
    and the page fell back to its list view with the model sitting right there.
    """
    from g1_stack.motor_probe import devserver
    from g1_stack.nodes import motor_probe as node_mod

    assert devserver.ASSETS == ASSETS
    assert node_mod.DEFAULT_ASSETS == ASSETS
    # And it has to be a real directory once the fetch script has run.
    assert ASSETS.name == "assets"
    assert ASSETS.parent.name == "g1_stack"


def test_index_is_served_at_the_root(live):
    base, _, _ = live
    status, body = fetch(base + "/")
    assert status == 200
    assert b"<!" in body[:200] or b"<html" in body[:400] or b"<div" in body


@pytest.mark.parametrize(
    "path",
    [
        "/app.js",
        "/style.css",
        "/vendor/three/three.module.js",
        "/vendor/urdf-loader/URDFLoader.js",
    ],
)
def test_static_assets_are_served(live, path):
    base, _, _ = live
    if path.startswith("/vendor/") and not (server_mod.STATIC_ROOT / path.lstrip("/")).is_file():
        pytest.skip("vendor/ is fetched, not committed")
    status, body = fetch(base + path)
    assert status == 200
    assert body, f"{path} served empty"


def test_the_page_references_the_urdf_the_fetch_script_installs(live):
    base, _, _ = live
    _, body = fetch(base + "/app.js")
    assert b"g1_29dof_with_hand.urdf" in body


# --- the catalog ---------------------------------------------------------


def test_catalog_endpoint_matches_the_real_config(live):
    base, _, _ = live
    _, body = fetch(base + "/api/catalog")
    motors = json.loads(body)["motors"]
    assert len(motors) == 43
    assert {m["group"] for m in motors} == {"body", "left_hand", "right_hand"}
    assert all(m["urdf_joint"].endswith("_joint") for m in motors)


# --- the fetched robot description --------------------------------------


@pytest.mark.skipif(not ASSETS.is_dir(), reason="run scripts/fetch_g1_description.sh")
def test_urdf_and_a_mesh_are_served_from_assets(live):
    base, _, _ = live
    status, urdf = fetch(base + "/assets/g1_29dof_with_hand.urdf")
    assert status == 200
    assert b"<robot" in urdf
    # The URDF points at meshes/*.STL relative to itself; that must resolve too.
    status, mesh = fetch(base + "/assets/meshes/pelvis.STL")
    assert status == 200
    assert len(mesh) > 1000


@pytest.mark.skipif(not ASSETS.is_dir(), reason="run scripts/fetch_g1_description.sh")
def test_assets_cannot_escape_the_assets_root(live):
    base, _, _ = live
    # Even a crafted path stays inside; the worst case is a 404.
    try:
        status, _ = fetch(base + "/assets/../../../../etc/passwd")
    except urllib.error.HTTPError as exc:
        status = exc.code
    assert status == 404


# --- the websocket deadman ----------------------------------------------


def ws_connect(address):
    """Do the RFC 6455 handshake by hand and return the raw socket."""
    sock = socket.create_connection(address, timeout=5)
    key = base64.b64encode(os.urandom(16)).decode()
    request = (
        "GET /api/stream HTTP/1.1\r\n"
        f"Host: {address[0]}:{address[1]}\r\n"
        "Upgrade: websocket\r\nConnection: Upgrade\r\n"
        f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n"
    )
    sock.sendall(request.encode())
    header = b""
    while b"\r\n\r\n" not in header:
        chunk = sock.recv(1)
        if not chunk:
            raise AssertionError("server closed during handshake")
        header += chunk
    assert b"101" in header.split(b"\r\n")[0], header
    assert server_mod.accept_key(key).encode() in header
    return sock


def client_frame(payload):
    """A masked text frame, the only kind a browser may send."""
    mask = os.urandom(4)
    masked = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
    return bytes([0x81, 0x80 | len(payload)]) + mask + masked


def test_stream_handshake_then_ping_marks_the_session_alive(live):
    _, address, service = live
    assert service.commands()[0] is False, "dead before the page connects"
    sock = ws_connect(address)
    try:
        sock.sendall(client_frame(b'{"type":"ping"}'))
        # The ping is what makes commanding legal.
        for _ in range(200):
            if service.commands()[0]:
                break
            threading.Event().wait(0.01)
        assert service.commands()[0] is True, "a ping must revive the session"

        # And the server must be pushing snapshots back.
        sock.settimeout(3.0)
        opcode, payload = server_mod.read_message(sock.makefile("rb"))
        assert opcode == 0x1
        snapshot = json.loads(payload)
        assert len(snapshot["motors"]) == 43
        assert set(snapshot) >= {"t", "started", "limp", "selected", "motors"}
    finally:
        sock.close()


def test_closing_the_stream_stops_commanding(live):
    _, address, service = live
    sock = ws_connect(address)
    sock.sendall(client_frame(b'{"type":"ping"}'))
    for _ in range(200):
        if service.commands()[0]:
            break
        threading.Event().wait(0.01)
    assert service.commands()[0] is True

    # Dropping the socket is the deadman: it must stop commanding, not linger.
    sock.close()
    for _ in range(300):
        if not service.commands()[0]:
            break
        threading.Event().wait(0.01)
    assert service.commands()[0] is False, "a dropped tab must stop the robot"
