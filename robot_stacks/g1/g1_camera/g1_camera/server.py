"""Camera server, runs on the robot's onboard Jetson.

Serves JPEG frames on a ZMQ REP socket; see wire.py for the contract.

Both listen on `--bind` only, default the Jetson's robot-network address
(`wire.DEFAULT_HOST`), not on every interface.

`--preview-port` adds a stdlib HTTP view of the same frames: `/frame.jpg` and
`/health`. It answers from the cache while that is younger than `--max-age`, so
a preview open during a run faster than that touches neither the camera nor the
REP socket.
"""

from __future__ import annotations

import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from . import wire
from .sources import DEFAULT_DEVICE, V4L2Source, VideoHubSource, initialize_dds, list_devices


class CameraServer:
    """Serves JPEG frames on a ZMQ REP socket, in a background thread.

    `source` is any callable returning a BGR array, so this can be exercised
    without a camera. `bind` is the one address both sockets listen on.

    With `preview_port` set it also serves the same frames over HTTP, from a
    cache that is only refilled once it is older than `max_age_s`. The REP path
    refills the same cache, so while the run is faster than `max_age_s` the
    preview costs zero camera reads and stays out of the REP queue whose round
    trip the run is measuring. Below that it grabs its own frames and takes
    `_source_lock`, which the policy loop then waits on: raise `max_age_s` if a
    preview open during a run slows it.
    """

    def __init__(
        self,
        port=None,
        source=None,
        zmq_module=None,
        encoder=None,
        preview_port=None,
        max_age_s=0.2,
        source_name=None,
        clock=time.monotonic,
        bind=wire.DEFAULT_HOST,
    ):
        self._zmq = zmq_module or __import__("zmq")
        self._source = source if source is not None else V4L2Source()
        self._encode = encoder or _encode_jpeg
        self._clock = clock
        self.max_age_s = max_age_s
        self.source_name = source_name or _source_name(self._source)
        self.bind = bind

        self._ctx = self._zmq.Context.instance()
        self._socket = self._ctx.socket(self._zmq.REP)
        requested = wire.DEFAULT_PORT if port is None else port
        # port 0 asks the OS for a free one, so tests never collide.
        self.port = (
            self._socket.bind_to_random_port(f"tcp://{bind}")
            if requested == 0
            else (self._socket.bind(f"tcp://{bind}:{requested}"), requested)[1]
        )

        self._stop = threading.Event()
        # served and failures count ZMQ replies only; the preview moves neither.
        self.served = 0
        self.failures = 0
        # Every call of _source goes through this. Neither VideoHubSource nor
        # V4L2Source is thread-safe and the preview reads on its own thread.
        self._source_lock = threading.Lock()
        # (jpeg, stamp), replaced as one object so a reader never sees a torn pair.
        self._cached = None
        self._thread = threading.Thread(target=self._serve, daemon=True)

        self.preview_port = None
        self._preview = None
        self._preview_thread = None
        if preview_port is not None:
            self._bind_preview(preview_port)

    def _bind_preview(self, port):
        """Bind now so `preview_port` is known before `start`, as the REP port is."""
        self._preview = ThreadingHTTPServer((self.bind, port), _PreviewHandler)
        self._preview.camera = self
        self.preview_port = self._preview.server_address[1]
        self._preview_thread = threading.Thread(
            target=self._preview.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True
        )

    def start(self):
        self._thread.start()
        if self._preview_thread is not None:
            self._preview_thread.start()
        return self

    def grab_jpeg(self):
        """One fresh encoded frame, cached on the way out. Raises what the source raises."""
        with self._source_lock:
            jpeg = self._encode(self._source())
        self._cached = (jpeg, self._clock())
        return jpeg

    def cached_jpeg(self):
        """(jpeg, age in seconds) of the newest frame, or (None, None)."""
        cached = self._cached
        if cached is None:
            return None, None
        return cached[0], self._clock() - cached[1]

    def preview_jpeg(self):
        """What the preview serves: the cache while fresh, else a fresh grab.

        None only when there is no frame at all and the grab failed.
        """
        jpeg, age = self.cached_jpeg()
        if jpeg is not None and age < self.max_age_s:
            return jpeg
        try:
            return self.grab_jpeg()
        except Exception:  # noqa: BLE001
            return jpeg

    def health(self):
        _, age = self.cached_jpeg()
        return {
            "served": self.served,
            "failures": self.failures,
            "age_s": None if age is None else round(age, 3),
            "source": self.source_name,
        }

    def _serve(self):
        while not self._stop.is_set():
            if not self._socket.poll(50, self._zmq.POLLIN):
                continue
            self._socket.recv()
            try:
                jpeg = self.grab_jpeg()
            except Exception:  # noqa: BLE001
                # Not answering wedges the client's REQ socket. An empty reply
                # gives it a recoverable CameraTimeout instead.
                self.failures += 1
                self._socket.send_multipart([b"", b"", b""])
                continue
            self.served += 1
            self._socket.send_multipart([jpeg, b"", b""])

    def stop(self):
        if self._preview is not None:
            # shutdown() blocks forever unless serve_forever is actually running.
            if self._preview_thread.is_alive():
                self._preview.shutdown()
                self._preview_thread.join(timeout=2.0)
            self._preview.server_close()
        self._stop.set()
        if self._thread.ident is not None:
            self._thread.join(timeout=2.0)
        self._socket.close(linger=0)
        close = getattr(self._source, "close", None)
        if close:
            close()


class _PreviewHandler(BaseHTTPRequestHandler):
    """GET /frame.jpg -> JPEG, GET /health -> JSON, anything else -> 404."""

    def do_GET(self):  # noqa: N802 - the stdlib's spelling
        path = self.path.split("?", 1)[0]
        if path == "/frame.jpg":
            self._send_frame()
        elif path == "/health":
            self._send_json(200, self.server.camera.health())
        else:
            self._send_json(404, {"error": "not found"})

    def _send_frame(self):
        jpeg = self.server.camera.preview_jpeg()
        if not jpeg:
            self._send_body(503, "text/plain", b"no frame\n")
            return
        # no-store, or the browser pins the first frame and the preview freezes.
        self._send_body(200, "image/jpeg", jpeg, headers=(("Cache-Control", "no-store"),))

    def _send_json(self, status, payload):
        self._send_body(status, "application/json", json.dumps(payload).encode("utf-8"))

    def _send_body(self, status, content_type, body, headers=()):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        for name, value in headers:
            self.send_header(name, value)
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):
        """Silent: one access line per frame would bury the server's own output."""


def _source_name(source):
    """A label for /health: a function's name, or an instance's class name."""
    return getattr(source, "__name__", type(source).__name__)


def _encode_jpeg(bgr):
    import cv2

    ok, buffer = cv2.imencode(".jpg", bgr)
    if not ok:
        raise RuntimeError("JPEG encode failed")
    return buffer.tobytes()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--bind",
        default=wire.DEFAULT_HOST,
        help="the address to listen on (default: %(default)s, the Jetson on the robot network; "
        "0.0.0.0 is every interface)",
    )
    parser.add_argument("--port", type=int, default=wire.DEFAULT_PORT)
    parser.add_argument(
        "--source",
        choices=("videohub", "v4l2"),
        default="videohub",
        help="videohub is the only colour route on this robot; v4l2 reaches the infrared node only",
    )
    parser.add_argument(
        "--device", type=int, default=DEFAULT_DEVICE, help="V4L2 node; --source v4l2 only"
    )
    parser.add_argument("--width", type=int, default=wire.DEFAULT_SIZE[0])
    parser.add_argument("--height", type=int, default=wire.DEFAULT_SIZE[1])
    parser.add_argument(
        "--fit",
        choices=("crop", "squash"),
        default="crop",
        help="videohub serves 16:9; crop takes the centre, squash distorts",
    )
    parser.add_argument(
        "--preview-port",
        type=int,
        default=wire.DEFAULT_PREVIEW_PORT,
        help="HTTP preview: /frame.jpg and /health",
    )
    parser.add_argument(
        "--no-preview", action="store_true", help="do not open the HTTP preview port"
    )
    parser.add_argument(
        "--max-age",
        type=float,
        default=0.2,
        help="seconds a preview frame may be stale before the preview grabs "
        "its own; raise it if the preview slows a run",
    )
    parser.add_argument(
        "--dds-iface", default=None, help="DDS interface, auto-detected on the robot itself"
    )
    parser.add_argument(
        "--list", action="store_true", help="print what each /dev/videoN is, and exit"
    )
    return parser.parse_args(argv)


def main():
    args = parse_args()

    if args.list:
        for name, description in list_devices():
            print(f"{name}: {description}")
        return

    size = (args.width, args.height)
    if args.source == "videohub":
        initialize_dds(args.dds_iface)
        source = VideoHubSource(fit=args.fit, size=size)
        origin = f"videohub, {args.fit} to {args.width}x{args.height}"
    else:
        source = V4L2Source(args.device, size=size)
        origin = f"/dev/video{args.device}"

    server = CameraServer(
        port=args.port,
        source=source,
        source_name=origin,
        preview_port=None if args.no_preview else args.preview_port,
        max_age_s=args.max_age,
        bind=args.bind,
    ).start()
    print(f"camera server on tcp://{args.bind}:{server.port} from {origin}; ctrl-c to stop")
    if server.preview_port is not None:
        print(f"preview on http://{args.bind}:{server.preview_port}/frame.jpg and /health")
    try:
        threading.Event().wait()
    except KeyboardInterrupt:
        server.stop()


if __name__ == "__main__":
    main()
