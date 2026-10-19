"""HTTP + WebSocket server for the motor probe. Stdlib only.

No aiohttp, no websockets package: this has to run on the on-board Python 3.8
where nothing can be pip-installed. The WebSocket half is RFC 6455's minimum --
handshake, text frames, close -- which is all `/api/stream` needs.

Knows nothing about ROS. It talks to a service object, so the whole HTTP
surface is testable with a fake.
"""

from __future__ import annotations

import base64
import hashlib
import json
import mimetypes
import posixpath
import struct
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

STATIC_ROOT = Path(__file__).resolve().parent / "static"

# RFC 6455 section 1.3.
_WS_GUID = b"258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

_OP_TEXT = 0x1
_OP_CLOSE = 0x8
_OP_PING = 0x9
_OP_PONG = 0xA

# How often the stream socket pushes a snapshot.
STREAM_PERIOD = 0.05


def accept_key(client_key: str) -> str:
    """The Sec-WebSocket-Accept value for a client's Sec-WebSocket-Key."""
    digest = hashlib.sha1(client_key.encode("ascii") + _WS_GUID).digest()
    return base64.b64encode(digest).decode("ascii")


def frame(payload: bytes, opcode: int = _OP_TEXT) -> bytes:
    """One unmasked server frame. Server frames are never masked."""
    header = bytes([0x80 | opcode])
    length = len(payload)
    if length < 126:
        header += bytes([length])
    elif length < (1 << 16):
        header += bytes([126]) + struct.pack(">H", length)
    else:
        header += bytes([127]) + struct.pack(">Q", length)
    return header + payload


def _read_exactly(stream, count: int) -> bytes:
    """`count` bytes, or b"" if the peer closed early."""
    chunks = []
    while count:
        chunk = stream.read(count)
        if not chunk:
            return b""
        chunks.append(chunk)
        count -= len(chunk)
    return b"".join(chunks)


def read_message(stream):
    """Next client frame as (opcode, payload), or (None, b"") when the peer goes away."""
    header = _read_exactly(stream, 2)
    if len(header) < 2:
        return None, b""
    opcode = header[0] & 0x0F
    masked = header[1] & 0x80
    length = header[1] & 0x7F
    if length == 126:
        extended = _read_exactly(stream, 2)
        if not extended:
            return None, b""
        length = struct.unpack(">H", extended)[0]
    elif length == 127:
        extended = _read_exactly(stream, 8)
        if not extended:
            return None, b""
        length = struct.unpack(">Q", extended)[0]
    mask = _read_exactly(stream, 4) if masked else b""
    if masked and not mask:
        return None, b""
    payload = _read_exactly(stream, length) if length else b""
    if length and not payload:
        return None, b""
    if masked:
        payload = bytes(byte ^ mask[i % 4] for i, byte in enumerate(payload))
    return opcode, payload


class ProbeHandler(BaseHTTPRequestHandler):
    """Routes /api/*, the WebSocket stream, /assets/* and the static page."""

    protocol_version = "HTTP/1.1"
    service = None
    assets_root = None

    def log_message(self, fmt, *args):
        """Quiet by default; a panel streaming at 20 Hz would flood the console."""

    # --- plumbing ------------------------------------------------------

    def _send_json(self, payload, status=200):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self):
        length = int(self.headers.get("Content-Length") or 0)
        if not length:
            return {}
        try:
            return json.loads(self.rfile.read(length).decode("utf-8"))
        except ValueError:
            return None

    def _send_file(self, path):
        if path is None or not path.is_file():
            self._send_json({"error": "not found"}, status=404)
            return
        body = path.read_bytes()
        content_type = mimetypes.guess_type(str(path))[0] or "application/octet-stream"
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _safe_join(self, root, relative):
        """`root/relative`, or None if it escapes root."""
        clean = posixpath.normpath("/" + relative).lstrip("/")
        candidate = (root / clean).resolve()
        try:
            candidate.relative_to(root.resolve())
        except ValueError:
            return None
        return candidate

    # --- routes --------------------------------------------------------

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/api/stream":
            self._serve_stream()
        elif path == "/api/catalog":
            self._send_json({"motors": self.service.catalog_json()})
        elif path.startswith("/assets/"):
            if self.assets_root is None:
                self._send_json({"error": "no assets directory"}, status=404)
                return
            self._send_file(self._safe_join(self.assets_root, path[len("/assets/") :]))
        else:
            relative = "index.html" if path == "/" else path.lstrip("/")
            self._send_file(self._safe_join(STATIC_ROOT, relative))

    def do_POST(self):
        path = self.path.split("?", 1)[0]
        body = self._read_json()
        if body is None:
            self._send_json({"ok": False, "message": "malformed JSON"}, status=400)
            return
        try:
            self._send_json(self._dispatch_post(path, body))
        except KeyError as exc:
            self._send_json({"ok": False, "message": f"no such route or field: {exc}"}, 404)
        except (ValueError, TypeError) as exc:
            self._send_json({"ok": False, "message": str(exc)}, status=400)

    def _dispatch_post(self, path, body):
        if path == "/api/control/start":
            ok, message = self.service.start()
            return {"ok": ok, "message": message}
        if path == "/api/control/stop":
            ok, message = self.service.stop()
            return {"ok": ok, "message": message}
        if path == "/api/select":
            self.service.select(str(body["name"]))
            return {"ok": True}
        if path == "/api/target":
            return {"ok": True, "q": self.service.set_target(str(body["name"]), float(body["q"]))}
        if path == "/api/mode":
            self.service.set_mode(bool(body["limp"]))
            return {"ok": True}
        raise KeyError(path)

    # --- the stream ----------------------------------------------------

    def _serve_stream(self):
        """Upgrade to WebSocket, then push a snapshot every STREAM_PERIOD."""
        key = self.headers.get("Sec-WebSocket-Key")
        if not key:
            self._send_json({"error": "not a websocket handshake"}, status=400)
            return
        self.send_response(101)
        self.send_header("Upgrade", "websocket")
        self.send_header("Connection", "Upgrade")
        self.send_header("Sec-WebSocket-Accept", accept_key(key))
        self.end_headers()
        self.wfile.flush()

        done = threading.Event()
        writer = threading.Thread(target=self._push_snapshots, args=(done,), daemon=True)
        writer.start()
        try:
            self._read_beats(done)
        finally:
            done.set()
            writer.join(timeout=1.0)
            # The socket dying is the deadman: stop commanding immediately.
            self.service.on_stream_closed()

    def _read_beats(self, done):
        """Client -> server: every text message is a liveness beat."""
        while not done.is_set():
            opcode, payload = read_message(self.rfile)
            if opcode is None or opcode == _OP_CLOSE:
                return
            if opcode == _OP_PING:
                self._write(frame(payload, _OP_PONG))
            elif opcode == _OP_TEXT:
                self.service.ping()

    def _push_snapshots(self, done):
        while not done.wait(STREAM_PERIOD):
            try:
                self._write(frame(json.dumps(self.service.snapshot()).encode("utf-8")))
            except (OSError, ValueError):
                done.set()
                return

    def _write(self, data):
        with self.server.write_lock:
            self.wfile.write(data)
            self.wfile.flush()


class ProbeServer(ThreadingHTTPServer):
    """Threading server carrying the service and a write lock for the stream."""

    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, service, assets_root=None):
        self.write_lock = threading.Lock()
        handler = type(
            "BoundProbeHandler",
            (ProbeHandler,),
            {"service": service, "assets_root": assets_root},
        )
        super().__init__(address, handler)


def serve(service, host="127.0.0.1", port=8080, assets_root=None):
    """Build the server. The caller runs `serve_forever`, so tests can drive it."""
    return ProbeServer((host, port), service, assets_root=assets_root)
