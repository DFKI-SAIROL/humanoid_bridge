"""Recorded runs on disk: one npz per probing motion (runs/) and per bridge session (sessions/).

Files are written by a background thread (compression must not stall the 20 ms control tick)
to <name>.part and renamed when complete, so listings never show half-written files. Each npz
holds per-tick arrays plus a `meta_json` string. downloaded.json remembers which files the web
commander has served in full.

File names use the browser's clock once the page has sent it (the robot's clock is not set).
"""
import io
import json
import os
import queue
import re
import shutil
import threading
import time
import zipfile
from datetime import datetime, timedelta, timezone
from pathlib import Path

import numpy as np

KINDS = ("runs", "sessions")
NAME_RE = re.compile(r"^[A-Za-z0-9_.\-]+\.npz$")


class RunStore:
    def __init__(self, root):
        self.root = Path(root)
        for kind in KINDS:
            (self.root / kind).mkdir(parents=True, exist_ok=True)
        self.clock_offset = 0.0
        self.clock_set = False
        self.tz = None  # robot's local zone until the browser sends its own
        self._lock = threading.Lock()
        self._meta_cache = {}
        self._downloaded_path = self.root / "downloaded.json"
        try:
            self._downloaded = json.loads(self._downloaded_path.read_text())
        except (OSError, ValueError):
            self._downloaded = {}
        self._seq = 1 + max([int(p.name.split("_")[0]) for kind in KINDS
                             for p in (self.root / kind).glob("*.npz") if p.name.split("_")[0].isdigit()] or [0])
        self._queue = queue.Queue()
        threading.Thread(target=self._writer, daemon=True).start()

    # ---------------------------------------------------------------- clock
    def set_clock(self, epoch_s, tz_offset_min=None):
        """tz_offset_min: JavaScript getTimezoneOffset() (minutes, UTC minus local)."""
        self.clock_offset = float(epoch_s) - time.time()
        if tz_offset_min is not None:
            self.tz = timezone(timedelta(minutes=-float(tz_offset_min)))
        self.clock_set = True

    def now(self):
        return time.time() + self.clock_offset

    def iso(self, t):
        return datetime.fromtimestamp(t, self.tz).isoformat(timespec="seconds")

    def stamp(self):
        return datetime.fromtimestamp(self.now(), self.tz).strftime("%Y%m%d-%H%M%S")

    # ---------------------------------------------------------------- writing
    def save(self, kind, label, ticks, meta, sync=False):
        """Queue `ticks` (list of dicts of equal-shaped values) for writing; returns the file name."""
        assert kind in KINDS
        with self._lock:
            seq = self._seq
            self._seq += 1
        stamp = self.stamp()
        name = f"{seq:04d}_{stamp}_{re.sub(r'[^A-Za-z0-9_.-]', '-', label)}.npz"
        meta = dict(meta, kind=kind, file=name, clock_from_browser=self.clock_set)
        job = (self.root / kind / name, ticks, meta)
        if sync:
            self._write(*job)
        else:
            self._queue.put(job)
        return name

    def flush(self):
        self._queue.join()

    def _writer(self):
        while True:
            job = self._queue.get()
            try:
                self._write(*job)
            except Exception as e:  # keep the writer alive; the error shows up in the client log
                print(f"[run_store] failed to write {job[0]}: {e}", flush=True)
            finally:
                self._queue.task_done()

    @staticmethod
    def _write(path, ticks, meta):
        arrays = {key: np.stack([np.asarray(t[key]) for t in ticks]) for key in ticks[0]}
        part = path.with_name(path.name + ".part")
        with open(part, "wb") as f:
            np.savez_compressed(f, meta_json=np.array(json.dumps(meta)), **arrays)
        os.replace(part, path)

    # ---------------------------------------------------------------- reading
    def path_of(self, name):
        if not NAME_RE.match(name):
            return None
        for kind in KINDS:
            p = self.root / kind / name
            if p.is_file():
                return p
        return None

    def _meta(self, path):
        st = path.stat()
        key = (path.name, st.st_mtime)
        if key not in self._meta_cache:
            try:
                with np.load(path) as z:
                    self._meta_cache[key] = json.loads(str(z["meta_json"]))
            except Exception:
                self._meta_cache[key] = {}
        return self._meta_cache[key]

    def list(self):
        entries = []
        for kind in KINDS:
            for p in sorted((self.root / kind).glob("*.npz")):
                m = self._meta(p)
                entries.append({
                    "name": p.name, "kind": kind[:-1], "bytes": p.stat().st_size,
                    "motion_id": m.get("motion_id"), "outcome": m.get("outcome"),
                    "start": m.get("start_iso"), "duration_s": m.get("duration_s"),
                    "n_ticks": m.get("n_ticks"), "max_tilt": m.get("max_tilt"),
                    "downloaded": self._downloaded.get(p.name),
                })
        entries.sort(key=lambda e: e["name"], reverse=True)
        return {"entries": entries, "free_bytes": shutil.disk_usage(self.root).free,
                "pending_writes": self._queue.unfinished_tasks}

    def mark_downloaded(self, names):
        with self._lock:
            stamp = self.iso(self.now())
            for n in names:
                self._downloaded[n] = stamp
            self._save_downloaded()

    def delete(self, names):
        deleted = []
        with self._lock:
            for n in names:
                p = self.path_of(n)
                if p is not None:
                    p.unlink()
                    self._downloaded.pop(n, None)
                    deleted.append(n)
            self._save_downloaded()
        return deleted

    def _save_downloaded(self):
        tmp = self._downloaded_path.with_suffix(".tmp")
        tmp.write_text(json.dumps(self._downloaded, indent=1))
        os.replace(tmp, self._downloaded_path)

    def zip_bytes(self, names):
        """Zip (stored: npz is already compressed) with runs/ and sessions/ kept apart."""
        buf = io.BytesIO()
        with zipfile.ZipFile(buf, "w", zipfile.ZIP_STORED) as z:
            for n in names:
                p = self.path_of(n)
                if p is not None:
                    z.write(p, f"{p.parent.name}/{p.name}")
        return buf.getvalue()
