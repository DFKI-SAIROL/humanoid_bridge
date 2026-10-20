"""Web commander for G1_probe.py: Control tab (HOME / Stand / motions / Damping / Stop bridge)
and Data tab (recorded runs and sessions: download to the browser's machine, delete).

Stdlib only (http.server in daemon threads). Control commands only go onto the client's queue --
the same path as the terminal -- so every check in G1_probe.py applies.

  GET  /                      the page
  GET  /api/status            JSON snapshot (mode, reference, robot health, motions, events)
  POST /api/cmd               {"cmd": "home" | "go" | "m <motion_id>" | "e" | "stop"}
  POST /api/clock             {"epoch_ms", "tz_offset_min"}  browser clock + zone for file names (robot clock is unset)
  GET  /api/files             recorded files (runs/ and sessions/) with metadata
  GET  /api/file/<name>       one npz (marked downloaded once fully sent)
  GET  /api/zip?names=a,b     several as one zip (marked downloaded once fully sent)
  POST /api/delete            {"names": [...]}
"""
import collections
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlparse

ALLOWED = ("home", "go", "e", "stop")
MAX_CMD_AGE_S = 1.0
PAGE = (Path(__file__).parent / "web_commander.html").read_bytes()


def serve(client, host, port):
    store = client.store
    # Per page: browser clock minus local monotonic clock, from its status polls. The largest
    # recent value is the one with the least network delay; a command's age is then measured in
    # that page's own clock, so it works for any device whatever its clock setting.
    page_offsets = collections.defaultdict(lambda: collections.deque(maxlen=50))

    def command_age(body):
        offs = page_offsets.get(str(body.get("page", "")))
        if not offs or not body.get("sent_ms"):
            return None
        return time.monotonic() - (float(body["sent_ms"]) / 1000.0 - max(offs))

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def handle(self):
            try:
                super().handle()
            except (BrokenPipeError, ConnectionResetError):
                pass  # browser closed the tab or reloaded mid-response

        def _send(self, code, body, ctype="application/json", filename=None):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            if filename:
                self.send_header("Content-Disposition", f'attachment; filename="{filename}"')
            self.end_headers()
            self.wfile.write(body)

        def _json(self, code, obj):
            self._send(code, json.dumps(obj).encode())

        def _body(self):
            n = int(self.headers.get("Content-Length", 0))
            return json.loads(self.rfile.read(n))

        def do_GET(self):
            url = urlparse(self.path)
            if url.path == "/":
                self._send(200, PAGE, "text/html; charset=utf-8")
            elif url.path == "/api/status":
                qs = parse_qs(url.query)
                if "page" in qs and "t" in qs:
                    page_offsets[qs["page"][0]].append(float(qs["t"][0]) / 1000.0 - time.monotonic())
                self._json(200, client.status())
            elif url.path == "/api/files":
                self._json(200, store.list())
            elif url.path.startswith("/api/file/"):
                name = unquote(url.path[len("/api/file/"):])
                path = store.path_of(name)
                if path is None:
                    self._json(404, {"ok": False, "error": "no such file"})
                    return
                self._send(200, path.read_bytes(), "application/octet-stream", filename=name)
                store.mark_downloaded([name])
            elif url.path == "/api/zip":
                names = [n for n in parse_qs(url.query).get("names", [""])[0].split(",") if store.path_of(n)]
                if not names:
                    self._json(400, {"ok": False, "error": "no files"})
                    return
                self._send(200, store.zip_bytes(names), "application/zip", filename=f"g1_probe_data_{store.stamp()}.zip")
                store.mark_downloaded(names)
            else:
                self._json(404, {})

        def do_POST(self):
            try:
                body = self._body()
            except (ValueError, TypeError):
                self._json(400, {"ok": False, "error": "bad request"})
                return
            if self.path == "/api/cmd":
                cmd = str(body.get("cmd", "")).strip()
                parts = cmd.split()
                ok = cmd in ALLOWED or (len(parts) == 2 and parts[0] == "m" and parts[1] in client.core.trained)
                if not ok:
                    self._json(400, {"ok": False, "error": "unknown command"})
                    return
                # A click that sat in a dropped connection must not run late (Stand arriving seconds
                # after it was pressed). Damping and stop always run.
                age = command_age(body)
                if cmd not in ("e", "stop") and age is not None and age > MAX_CMD_AGE_S:
                    client._say("WARN", f"'{cmd}' arrived {age:.1f} s after it was sent: NOT executed")
                    self._json(409, {"ok": False, "error": f"arrived {age:.1f} s late, not executed"})
                    return
                client.commands.put(cmd)
                self._json(200, {"ok": True})
            elif self.path == "/api/clock":
                if not store.clock_set:
                    store.set_clock(float(body["epoch_ms"]) / 1000.0, body.get("tz_offset_min"))
                self._json(200, {"ok": True})
            elif self.path == "/api/delete":
                names = [str(n) for n in body.get("names", [])]
                self._json(200, {"ok": True, "deleted": store.delete(names)})
            else:
                self._json(404, {})

    server = ThreadingHTTPServer((host, port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server
