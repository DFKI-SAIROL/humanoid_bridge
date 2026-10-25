"""The robot's camera as a g1_camera source: rendered on its own thread, read as BGR."""

import threading
import time

import mujoco


class SimCamera:
    """Renders the robot's camera at `hz`; calling it returns the newest BGR frame."""

    def __init__(self, sim, lock, size=(640, 480), hz=30.0):
        self.sim = sim
        self.lock = lock
        self.size = size
        self.period = 1.0 / hz
        self.frame = None
        self.error = None
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)

    def start(self, timeout=10.0):
        """Starts rendering; raises what the GL context raised, or on no first frame."""
        self._thread.start()
        self._ready.wait(timeout)
        if self.error is not None:
            raise self.error
        if self.frame is None:
            raise RuntimeError(f"no camera frame within {timeout} s")
        return self

    def _loop(self):
        width, height = self.size
        try:
            # The GL context belongs to the thread that made it.
            renderer = mujoco.Renderer(self.sim.model, height=height, width=width)
        except Exception as exc:  # noqa: BLE001
            self.error = exc
            self._ready.set()
            return
        due = time.monotonic()
        while not self._stop.is_set():
            with self.lock:
                renderer.update_scene(self.sim.data, camera=self.sim.robot.camera)
            self.frame = renderer.render()[:, :, ::-1].copy()  # RGB -> BGR, as cv2 sources
            self._ready.set()
            due = max(due + self.period, time.monotonic())
            time.sleep(max(0.0, due - time.monotonic()))
        renderer.close()

    def __call__(self):
        if self.frame is None:
            raise RuntimeError("no camera frame yet")
        return self.frame

    def close(self):
        self._stop.set()
        self._thread.join(timeout=2.0)
