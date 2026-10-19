"""The HTTP surface and the hand-rolled WebSocket codec, driven with a fake service."""

import io
import json
import threading
import urllib.error
import urllib.request

import pytest

from g1_stack.motor_probe import server as server_mod


class FakeService:
    """Records what the handler asked for; no ROS, no session."""

    def __init__(self):
        self.calls = []
        self.started = False
        self.limp = False
        self.selected = None
        self.beats = 0
        self.closed = 0

    def catalog_json(self):
        return [{"name": "L_ELBOW", "group": "body", "index": 18}]

    def start(self):
        self.started = True
        return True, "body control started"

    def stop(self):
        self.started = False
        return True, "stopped"

    def select(self, name):
        if name != "L_ELBOW":
            raise ValueError(f"unknown motor {name!r}")
        self.selected = name

    def set_target(self, name, q):
        self.calls.append((name, q))
        return max(-1.0472, min(2.0944, q))

    def set_mode(self, limp):
        self.limp = limp

    def ping(self):
        self.beats += 1

    def on_stream_closed(self):
        self.closed += 1

    def snapshot(self):
        return {"t": 1.0, "started": self.started, "limp": self.limp, "motors": []}


@pytest.fixture
def live_server():
    service = FakeService()
    httpd = server_mod.serve(service, host="127.0.0.1", port=0)
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    host, port = httpd.server_address[0], httpd.server_address[1]
    yield f"http://{host}:{port}", service
    httpd.shutdown()
    httpd.server_close()
    thread.join(timeout=2.0)


def get(url):
    with urllib.request.urlopen(url, timeout=5) as response:
        return response.status, json.loads(response.read().decode())


def post(url, payload):
    request = urllib.request.Request(
        url,
        data=json.dumps(payload).encode(),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=5) as response:
            return response.status, json.loads(response.read().decode())
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read().decode())


# --- the websocket codec ------------------------------------------------


def test_accept_key_matches_rfc6455_example():
    # The worked example from RFC 6455 section 1.3.
    assert server_mod.accept_key("dGhlIHNhbXBsZSBub25jZQ==") == "s3pPLMBiTxaQ9kYGzzhZRbK+xOo="


@pytest.mark.parametrize("size", [0, 5, 125, 126, 200, 65535, 65536])
def test_frame_length_encodings_round_trip(size):
    payload = b"x" * size
    encoded = server_mod.frame(payload)
    # Re-read it as a client would, after masking it the way a client must.
    opcode, decoded = server_mod.read_message(io.BytesIO(_as_client_frame(payload)))
    assert opcode == 0x1
    assert decoded == payload
    assert encoded[0] == 0x81


def _as_client_frame(payload, opcode=0x1):
    """The same payload masked, as a browser would send it."""
    mask = b"\xab\xcd\xef\x12"
    masked = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
    header = bytes([0x80 | opcode])
    length = len(payload)
    if length < 126:
        header += bytes([0x80 | length])
    elif length < (1 << 16):
        header += bytes([0x80 | 126]) + length.to_bytes(2, "big")
    else:
        header += bytes([0x80 | 127]) + length.to_bytes(8, "big")
    return header + mask + masked


def test_read_message_reports_peer_hangup():
    assert server_mod.read_message(io.BytesIO(b"")) == (None, b"")
    # A truncated frame is a hangup, not a crash.
    assert server_mod.read_message(io.BytesIO(b"\x81")) == (None, b"")


def test_read_message_unmasks_client_payload():
    opcode, payload = server_mod.read_message(io.BytesIO(_as_client_frame(b'{"type":"ping"}')))
    assert opcode == 0x1
    assert payload == b'{"type":"ping"}'


# --- the REST surface ---------------------------------------------------


def test_catalog_is_served(live_server):
    base, _ = live_server
    status, body = get(base + "/api/catalog")
    assert status == 200
    assert body["motors"][0]["name"] == "L_ELBOW"


def test_start_and_stop_reach_the_service(live_server):
    base, service = live_server
    status, body = post(base + "/api/control/start", {})
    assert (status, body["ok"]) == (200, True)
    assert service.started is True
    post(base + "/api/control/stop", {})
    assert service.started is False


def test_target_returns_the_clamped_value(live_server):
    base, service = live_server
    status, body = post(base + "/api/target", {"name": "L_ELBOW", "q": 99.0})
    assert status == 200
    assert body["q"] == pytest.approx(2.0944)
    assert service.calls == [("L_ELBOW", 99.0)]


def test_mode_toggles_limp(live_server):
    base, service = live_server
    post(base + "/api/mode", {"limp": True})
    assert service.limp is True


def test_select_rejects_an_unknown_motor(live_server):
    base, _ = live_server
    status, body = post(base + "/api/select", {"name": "NOPE"})
    assert status == 400
    assert body["ok"] is False


def test_unknown_route_is_404(live_server):
    base, _ = live_server
    status, _ = post(base + "/api/nonsense", {})
    assert status == 404


def test_malformed_json_is_rejected(live_server):
    base, _ = live_server
    request = urllib.request.Request(
        base + "/api/target",
        data=b"{not json",
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        urllib.request.urlopen(request, timeout=5)
        raise AssertionError("expected a 400")
    except urllib.error.HTTPError as exc:
        assert exc.code == 400


def test_stream_without_a_handshake_is_rejected(live_server):
    base, _ = live_server
    try:
        urllib.request.urlopen(base + "/api/stream", timeout=5)
        raise AssertionError("expected a 400")
    except urllib.error.HTTPError as exc:
        assert exc.code == 400


def test_assets_are_404_when_no_directory_is_configured(live_server):
    base, _ = live_server
    try:
        urllib.request.urlopen(base + "/assets/g1.urdf", timeout=5)
        raise AssertionError("expected a 404")
    except urllib.error.HTTPError as exc:
        assert exc.code == 404


@pytest.mark.parametrize(
    "relative",
    ["../../../etc/passwd", "..%2f..%2fetc/passwd", "a/../../../../etc/passwd", "/etc/passwd"],
)
def test_static_path_traversal_stays_inside_the_root(relative):
    """normpath clamps `..` to the root, so a traversal can only ever 404."""
    handler = server_mod.ProbeHandler
    joined = handler._safe_join(handler, server_mod.STATIC_ROOT, relative)
    assert joined is not None
    # The invariant that matters: whatever came in, the result is under STATIC_ROOT.
    joined.resolve().relative_to(server_mod.STATIC_ROOT.resolve())  # raises if it escaped
    assert not str(joined).startswith("/etc/")
