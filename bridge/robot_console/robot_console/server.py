"""HTTP + WebSocket server for the console. Stdlib only.

The WebSocket half is RFC 6455's minimum -- handshake, text frames, close --
which is all `/api/stream` needs. Knows nothing about ROS: it talks to a
service object, so the whole HTTP surface is testable with a fake.
Routes and rules: docs/robot_console.md.
"""

from __future__ import annotations

import base64
import hashlib
import json
import mimetypes
import posixpath
import struct
import sys
import threading
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

from . import auth
from .service import Refused

STATIC_ROOT = Path(__file__).resolve().parent / "static"
DEFAULT_VENDOR_ROOT = Path("~/.local/share/robot_console/vendor").expanduser()

# RFC 6455 section 1.3.
_WS_GUID = b"258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

_OP_TEXT = 0x1
_OP_CLOSE = 0x8
_OP_PING = 0x9
_OP_PONG = 0xA

# How often the stream socket pushes a snapshot: 20 Hz keeps the 3D view fluid.
STREAM_PERIOD = 0.05
CAMERA_TIMEOUT = 2.0
CAMERA_MAX_BYTES = 8 * 1024 * 1024

_NO_TOKEN_PAGE = b"""<!DOCTYPE html><meta charset="utf-8"><title>Robot console</title>
<p>Open the URL with the token that <code>robot_console</code> printed at start
(<code>http://host:port/?token=...</code>). The token is in
<code>~/.config/robot_console/token</code> on the console machine.</p>"""


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


class ConsoleHandler(BaseHTTPRequestHandler):
    """Routes /api/*, the WebSocket stream, /assets/*, /vendor/* and the static page."""

    protocol_version = "HTTP/1.1"
    service = None
    token = None
    assets_root = None
    vendor_root = None
    camera_url = None

    def log_message(self, fmt, *args):
        """Quiet: a stream at 10 Hz would flood the terminal."""

    # --- plumbing ------------------------------------------------------

    def _send_bytes(self, body, content_type, status=200, extra=()):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        for key, value in extra:
            self.send_header(key, value)
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(body)

    def _send_json(self, payload, status=200):
        self._send_bytes(json.dumps(payload).encode("utf-8"), "application/json", status)

    def _read_json(self):
        length = int(self.headers.get("Content-Length") or 0)
        if not length:
            return {}
        if length > 64 * 1024:
            return None
        try:
            body = json.loads(self.rfile.read(length).decode("utf-8"))
        except ValueError:
            return None
        return body if isinstance(body, dict) else None

    def _send_file(self, path):
        if path is None or not path.is_file():
            self._send_json({"error": "not found"}, status=404)
            return
        content_type = mimetypes.guess_type(str(path))[0] or "application/octet-stream"
        if path.suffix in (".js", ".mjs"):
            content_type = "text/javascript"
        self._send_bytes(path.read_bytes(), content_type)

    @staticmethod
    def _safe_join(root, relative):
        """`root/relative`, or None if it escapes root."""
        if root is None:
            return None
        clean = posixpath.normpath("/" + relative).lstrip("/")
        candidate = (Path(root) / clean).resolve()
        try:
            candidate.relative_to(Path(root).resolve())
        except ValueError:
            return None
        return candidate

    def _authorised(self) -> bool:
        return auth.token_ok(self.headers, self.token)

    # --- routes --------------------------------------------------------

    def do_GET(self):
        split = urlsplit(self.path)
        path = split.path

        if path == "/" and auth.query_token_ok(split.query, self.token):
            # Trade the URL token for a cookie, and drop it from the address bar.
            self._send_bytes(
                b"",
                "text/plain",
                303,
                (("Location", "/"), ("Set-Cookie", auth.cookie_header(self.token))),
            )
            return
        if not self._authorised():
            if path in ("/", "/index.html"):
                self._send_bytes(_NO_TOKEN_PAGE, "text/html; charset=utf-8", 401)
            else:
                self._send_json({"error": "no token"}, status=401)
            return

        query = parse_qs(split.query)
        if path == "/api/stream":
            if not auth.origin_ok(self.headers):
                self._send_json({"error": "cross-origin"}, status=403)
                return
            self._serve_stream()
        elif path == "/api/catalog":
            self._send_json({"motors": self.service.catalog_json()})
        elif path == "/api/process/log":
            try:
                name = query["name"][0]
                since = int(query.get("since", ["0"])[0])
                self._send_json(self.service.process_log(name, since))
            except (KeyError, ValueError) as exc:
                self._send_json({"error": f"bad log request: {exc}"}, status=400)
            except Refused as exc:
                self._send_json({"error": str(exc)}, status=409)
        elif path == "/api/camera/frame.jpg":
            self._proxy_camera()
        elif path.startswith("/assets/"):
            self._send_file(self._safe_join(self.assets_root, path[len("/assets/") :]))
        elif path.startswith("/vendor/"):
            self._send_file(self._safe_join(self.vendor_root, path[len("/vendor/") :]))
        else:
            relative = "index.html" if path == "/" else path.lstrip("/")
            self._send_file(self._safe_join(STATIC_ROOT, relative))

    def do_POST(self):
        if not self._authorised():
            self._send_json({"ok": False, "message": "no token"}, status=401)
            return
        if not auth.origin_ok(self.headers):
            self._send_json({"ok": False, "message": "cross-origin request refused"}, status=403)
            return
        path = urlsplit(self.path).path
        body = self._read_json()
        if body is None:
            self._send_json({"ok": False, "message": "malformed JSON"}, status=400)
            return
        try:
            self._send_json(self._dispatch_post(path, body))
        except Refused as exc:
            self._send_json({"ok": False, "message": str(exc)}, status=409)
        except KeyError as exc:
            self._send_json({"ok": False, "message": f"no such route or field: {exc}"}, 404)
        except (ValueError, TypeError) as exc:
            self._send_json({"ok": False, "message": str(exc)}, status=400)

    def _dispatch_post(self, path, body):
        service = self.service
        client = body.get("client")

        def result(pair):
            ok, message = pair
            return {"ok": ok, "message": message}

        # Any tab: these only stop things.
        if path == "/api/estop":
            return result(service.estop(str(body["mode"])))
        if path == "/api/stop":
            return result(service.stop())
        if path == "/api/process/stop":
            return result(service.process_stop(str(body["name"])))
        if path == "/api/jog/stop":
            return result(service.jog_stop())
        if path == "/api/control/take":
            return result(service.take_control(str(client)))
        # The controller only.
        if path == "/api/reset":
            return result(service.reset(client))
        if path == "/api/arm":
            return result(service.arm(client, body.get("confirm")))
        if path == "/api/ready":
            return result(service.ready(client, body.get("confirm")))
        if path == "/api/process/start":
            params = body.get("params") or {}
            if not isinstance(params, dict):
                raise ValueError("params: expected an object")
            return result(service.process_start(client, str(body["name"]), params))
        if path == "/api/process/answer":
            return result(service.process_answer(client, str(body["name"]), str(body["answer"])))
        if path == "/api/jog/start":
            return result(service.jog_start(client))
        if path == "/api/jog/select":
            service.jog_select(client, str(body["name"]))
            return {"ok": True}
        if path == "/api/jog/target":
            return {
                "ok": True,
                "q": service.jog_target(client, str(body["name"]), float(body["q"])),
            }
        if path == "/api/jog/mode":
            service.jog_mode(client, bool(body["limp"]))
            return {"ok": True}
        if path == "/api/jog/speed":
            service.jog_speed(client, float(body["speed"]))
            return {"ok": True}
        raise KeyError(path)

    def _proxy_camera(self):
        """The configured camera JPEG; the URL never comes from the request."""
        if not self.camera_url:
            self._send_json({"error": "no camera configured"}, status=404)
            return
        try:
            with urllib.request.urlopen(self.camera_url, timeout=CAMERA_TIMEOUT) as reply:
                body = reply.read(CAMERA_MAX_BYTES + 1)
                content_type = reply.headers.get("Content-Type", "image/jpeg")
        except (urllib.error.URLError, OSError, ValueError) as exc:
            self._send_json({"error": f"camera: {exc}"}, status=502)
            return
        if len(body) > CAMERA_MAX_BYTES:
            self._send_json({"error": "camera frame too large"}, status=502)
            return
        self._send_bytes(body, content_type)

    # --- the stream ----------------------------------------------------

    def _serve_stream(self):
        """Upgrade to WebSocket, say hello, then push a snapshot every STREAM_PERIOD."""
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

        self._write_lock = threading.Lock()
        client = self.service.connect()
        done = threading.Event()
        try:
            self._write(frame(json.dumps({"type": "hello", "client": client}).encode("utf-8")))
            writer = threading.Thread(target=self._push_snapshots, args=(done, client), daemon=True)
            writer.start()
            self._read_messages(done, client)
        except (OSError, ValueError):
            pass
        finally:
            done.set()
            # The socket dying frees the control lock and trips the jog deadman.
            self.service.disconnect(client)

    def _read_messages(self, done, client):
        """Client -> server: `{"type": "ping"}` keeps the jog deadman alive."""
        while not done.is_set():
            opcode, payload = read_message(self.rfile)
            if opcode is None or opcode == _OP_CLOSE:
                return
            if opcode == _OP_PING:
                self._write(frame(payload, _OP_PONG))
            elif opcode == _OP_TEXT:
                try:
                    message = json.loads(payload.decode("utf-8"))
                except ValueError:
                    continue
                if isinstance(message, dict) and message.get("type") == "ping":
                    self.service.ping(client)

    def _push_snapshots(self, done, client):
        while not done.wait(STREAM_PERIOD):
            try:
                snapshot = dict(self.service.snapshot(client), type="state")
                self._write(frame(json.dumps(snapshot).encode("utf-8")))
            except (OSError, ValueError):
                done.set()
                return

    def _write(self, data):
        with self._write_lock:
            self.wfile.write(data)
            self.wfile.flush()


class ConsoleServer(ThreadingHTTPServer):
    """Threading server bound to one service and one token."""

    daemon_threads = True
    allow_reuse_address = True

    def handle_error(self, request, client_address):
        """A dropped connection is normal (a phone locks its screen); log only real errors."""
        if isinstance(sys.exc_info()[1], (ConnectionError, TimeoutError)):
            return
        super().handle_error(request, client_address)


def serve(
    service,
    token,
    host="127.0.0.1",
    port=8080,
    assets_root=None,
    vendor_root=DEFAULT_VENDOR_ROOT,
    camera_url=None,
):
    """Build the server. The caller runs `serve_forever`, so tests can drive it."""
    handler = type(
        "BoundConsoleHandler",
        (ConsoleHandler,),
        {
            "service": service,
            "token": token,
            "assets_root": assets_root,
            "vendor_root": vendor_root,
            "camera_url": camera_url,
        },
    )
    return ConsoleServer((host, port), handler)
