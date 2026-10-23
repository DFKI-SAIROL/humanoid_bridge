"""The camera server against the real client, with no camera.

Drives `CameraServer` through `CameraClient`, so the wire contract is tested
from both ends rather than against a mock.
"""

from __future__ import annotations

import numpy as np
import pytest

from g1_camera import (
    CameraClient,
    CameraServer,
    CameraTimeout,
    VideoHubSource,
    fit_frame,
    synthetic_rgb,
)

LOOPBACK = "127.0.0.1"  # never the robot network, never every interface


def _decode(payload):
    """JPEG bytes to BGR, as VideoHubSource does internally."""
    import cv2

    return cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)


def bgr_frame(seed=3):
    """What a V4L2 capture hands back: the same image, channel-reversed."""
    return synthetic_rgb(seed)[:, :, ::-1].copy()


@pytest.fixture
def pair():
    server = CameraServer(bind=LOOPBACK, port=0, source=bgr_frame).start()
    client = CameraClient(host="127.0.0.1", port=server.port, timeout_ms=3000)
    yield server, client
    client.close()
    server.stop()


def test_a_frame_survives_the_round_trip_in_rgb(pair):
    """The channel order, end to end.

    A swap passes every shape check, so this compares against both orders:
    JPEG is lossy, but a swap is two orders of magnitude worse than that.
    """
    _, client = pair
    rgb = client.read_rgb()
    assert rgb.shape == (480, 640, 3)
    assert rgb.dtype == np.uint8

    expected = synthetic_rgb(3).astype(int)
    error = np.abs(rgb.astype(int) - expected).mean()
    if_swapped = np.abs(rgb.astype(int) - expected[:, :, ::-1]).mean()
    assert error < 5.0
    assert error < if_swapped / 10


def test_a_failed_grab_raises_camera_timeout_and_recovers(pair):
    """A camera that stops must not wedge the client.

    The server replies with empty parts rather than not answering, which the
    client turns into CameraTimeout. cv2.imdecode raises on an empty buffer,
    so the decoder special-cases it.
    """
    server, client = pair

    def broken():
        raise RuntimeError("camera gone")

    server._source = broken
    with pytest.raises(CameraTimeout):
        client.read_rgb()
    assert server.failures == 1

    server._source = bgr_frame
    assert client.read_rgb().shape == (480, 640, 3)


def test_list_devices_does_not_explode_without_a_camera():
    """Used on a new robot to find which /dev/videoN is the colour sensor."""
    from g1_camera import list_devices

    assert isinstance(list_devices(), list)


# --- the videohub source ---------------------------------------------------


class FakeVideoClient:
    """Stands in for the SDK's VideoClient: (code, jpeg bytes)."""

    def __init__(self, code=0, payload=b"", calls=0):
        self.code = code
        self.payload = payload
        self.calls = calls

    def GetImageSample(self):  # noqa: N802 - the SDK's spelling
        self.calls += 1
        return self.code, self.payload


def hd_jpeg(width=1920, height=1080):
    """A 16:9 JPEG with a distinct centre, as videohub serves."""
    import cv2

    # A 4:3 crop of 16:9 keeps 3/4 of the width, so the crop discards the
    # outer eighths.
    band = width // 8
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    frame[:, :] = (0, 255, 0)  # centre, green
    frame[:, :band] = (255, 0, 0)  # left edge, blue in BGR
    frame[:, width - band :] = (0, 0, 255)  # right edge, red in BGR
    ok, buffer = cv2.imencode(".jpg", frame)
    assert ok
    return buffer.tobytes()


def test_fit_frame_crops_to_the_default_shape_and_keeps_the_centre():
    """16:9 to 4:3: the coloured edges go, the middle survives. Content, not just shape."""
    frame = fit_frame(_decode(hd_jpeg()), mode="crop")
    assert frame.shape == (480, 640, 3)
    green, blue, red = frame[:, :, 1].mean(), frame[:, :, 0].mean(), frame[:, :, 2].mean()
    assert green > 200
    assert blue < 60 and red < 60


def test_fit_frame_squash_keeps_the_whole_field_of_view():
    frame = fit_frame(_decode(hd_jpeg()), mode="squash")
    assert frame.shape == (480, 640, 3)
    # The squashed frame still contains the coloured edges the crop discarded.
    # Each is an eighth of the width, so about 255/8 once averaged.
    assert frame[:, :, 0].mean() > 15
    assert frame[:, :, 2].mean() > 15


def test_fit_frame_rejects_an_unknown_mode():
    with pytest.raises(ValueError):
        fit_frame(_decode(hd_jpeg()), mode="stretch")


def test_videohub_source_returns_a_default_shaped_bgr_frame():
    source = VideoHubSource(client=FakeVideoClient(0, hd_jpeg()))
    frame = source()
    assert frame.shape == (480, 640, 3)
    assert frame.dtype == np.uint8


def test_videohub_source_raises_on_a_declined_sample():
    """A non-zero code must raise, not yield a stale or empty frame."""
    source = VideoHubSource(client=FakeVideoClient(3103, b""))
    with pytest.raises(RuntimeError, match="3103"):
        source()


def test_videohub_source_raises_on_an_undecodable_payload():
    source = VideoHubSource(client=FakeVideoClient(0, b"not a jpeg"))
    with pytest.raises(RuntimeError, match="undecodable"):
        source()


def test_the_server_serves_videohub_frames_through_the_real_client():
    """The whole path with videohub behind it, over ZMQ, no camera."""
    source = VideoHubSource(client=FakeVideoClient(0, hd_jpeg()))
    server = CameraServer(bind=LOOPBACK, port=0, source=source).start()
    client = CameraClient(host="127.0.0.1", port=server.port, timeout_ms=3000)
    try:
        rgb = client.read_rgb()
        assert rgb.shape == (480, 640, 3)
        # read_rgb flips to RGB, so the green centre is channel 1 either way.
        assert rgb[:, :, 1].mean() > 200
    finally:
        client.close()
        server.stop()


# --- the HTTP preview ------------------------------------------------------


class CountingSource:
    """Counts every grab, so a test can prove the preview did not take one."""

    def __init__(self):
        self.calls = 0

    def __call__(self):
        self.calls += 1
        return bgr_frame()


class FakeClock:
    """A clock the test moves by hand, so staleness never depends on timing."""

    def __init__(self, now=1000.0):
        self.now = now

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


def _broken():
    raise RuntimeError("camera gone")


def _get(server, path):
    """(status, headers, body) from the preview. 404 and 503 come back too."""
    from urllib.error import HTTPError
    from urllib.request import urlopen

    url = f"http://127.0.0.1:{server.preview_port}{path}"
    try:
        response = urlopen(url, timeout=5)
    except HTTPError as exc:
        return exc.code, dict(exc.headers), exc.read()
    with response:
        return response.getcode(), dict(response.headers), response.read()


@pytest.fixture
def preview():
    source = CountingSource()
    clock = FakeClock()
    server = CameraServer(bind=LOOPBACK, port=0, source=source, preview_port=0, clock=clock).start()
    yield server, source, clock
    server.stop()


def test_the_preview_serves_the_cached_frame_without_touching_the_camera(preview):
    """The load-bearing one: a preview left open must cost no camera access."""
    server, source, _ = preview
    status, headers, body = _get(server, "/frame.jpg")

    assert status == 200
    assert headers["Content-Type"] == "image/jpeg"
    # Without no-store the browser pins the first frame and the view freezes.
    assert headers["Cache-Control"] == "no-store"
    assert source.calls == 1

    for _ in range(3):
        assert _get(server, "/frame.jpg")[2] == body
    assert source.calls == 1


def test_a_frame_served_over_zmq_fills_the_preview_cache(preview):
    """While a run pulls frames the preview rides along on its cache."""
    server, source, _ = preview
    client = CameraClient(host="127.0.0.1", port=server.port, timeout_ms=3000)
    try:
        client.read_rgb()
        assert source.calls == 1
        assert _get(server, "/frame.jpg")[0] == 200
        assert source.calls == 1
    finally:
        client.close()


def test_a_stale_cache_is_refilled_from_the_camera(preview):
    server, source, clock = preview
    _get(server, "/frame.jpg")
    assert source.calls == 1

    clock.advance(server.max_age_s + 0.1)
    assert _get(server, "/frame.jpg")[0] == 200
    assert source.calls == 2


def test_a_failed_refill_still_serves_the_stale_frame(preview):
    """A wedged camera degrades the preview to an old frame, not to a 503."""
    server, _, clock = preview
    body = _get(server, "/frame.jpg")[2]

    clock.advance(server.max_age_s + 0.1)
    server._source = _broken
    status, _, served = _get(server, "/frame.jpg")
    assert status == 200
    assert served == body


def test_no_frame_and_a_failing_grab_is_a_503():
    server = CameraServer(bind=LOOPBACK, port=0, source=_broken, preview_port=0).start()
    try:
        status, _, body = _get(server, "/frame.jpg")
        assert status == 503
        assert b"no frame" in body
    finally:
        server.stop()


def test_health_reports_the_counters_the_age_and_the_source(preview):
    import json

    server, _, clock = preview
    status, headers, body = _get(server, "/health")
    assert status == 200
    assert headers["Content-Type"] == "application/json"
    assert json.loads(body) == {
        "served": 0,
        "failures": 0,
        "age_s": None,
        "source": "CountingSource",
        "capture_hz": None,
        "capture_failures": 0,
    }

    _get(server, "/frame.jpg")
    clock.advance(0.5)
    assert json.loads(_get(server, "/health")[2])["age_s"] == 0.5


def test_an_unknown_path_is_a_404(preview):
    server, _, _ = preview
    assert _get(server, "/")[0] == 404
    assert _get(server, "/frame.png")[0] == 404


def test_the_preview_is_off_unless_a_port_is_given():
    server = CameraServer(bind=LOOPBACK, port=0, source=bgr_frame).start()
    try:
        assert server.preview_port is None
    finally:
        server.stop()


def test_stop_does_not_hang_on_a_preview_that_never_started():
    """shutdown() blocks forever if serve_forever was never entered."""
    CameraServer(bind=LOOPBACK, port=0, source=bgr_frame, preview_port=0).stop()


def test_the_server_listens_on_the_bind_address_only():
    import zmq

    server = CameraServer(bind=LOOPBACK, port=0, source=bgr_frame, preview_port=0)
    try:
        endpoint = server._socket.getsockopt(zmq.LAST_ENDPOINT).decode()
        assert endpoint == f"tcp://{LOOPBACK}:{server.port}"
        assert server._preview.server_address[0] == LOOPBACK
    finally:
        server.stop()


# --- capture thread and stream ---------------------------------------------


@pytest.fixture
def capturing():
    source = CountingSource()
    server = CameraServer(
        bind=LOOPBACK, port=0, source=source, preview_port=0, capture_hz=50.0
    ).start()
    client = CameraClient(host="127.0.0.1", port=server.port, timeout_ms=3000)
    yield server, source, client
    client.close()
    server.stop()


def _until(predicate, seconds=3.0):
    import time

    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()


def test_capture_grabs_on_its_own_and_rep_answers_from_the_cache(capturing):
    server, source, client = capturing
    assert _until(lambda: source.calls >= 3)
    with server._source_lock:  # a grab in progress blocks the capture thread, not REP
        assert client.read_rgb().shape == (480, 640, 3)
    assert server.served == 1


def test_capture_answers_empty_once_the_cache_is_stale(capturing):
    """A dead camera under capture gives CameraTimeout, never the last good frame."""
    server, _, client = capturing
    assert _until(lambda: server.cached_jpeg()[0] is not None)
    server._source = _broken
    assert _until(lambda: server.cached_jpeg()[1] > server.max_age_s)
    with pytest.raises(CameraTimeout):
        client.read_rgb()
    assert server.capture_failures > 0


def _stream_parts(server, count):
    """The first `count` JPEG parts of /stream.mjpeg, and the Content-Type."""
    from urllib.request import urlopen

    response = urlopen(f"http://127.0.0.1:{server.preview_port}/stream.mjpeg", timeout=5)
    parts = []
    with response:
        kind = response.headers["Content-Type"]
        while len(parts) < count:
            assert response.readline() == b"--frame\r\n"
            headers = {}
            for line in iter(response.readline, b"\r\n"):
                name, _, value = line.decode().partition(":")
                headers[name.strip().lower()] = value.strip()
            parts.append(response.read(int(headers["content-length"])))
            assert response.readline() == b"\r\n"
    return kind, parts


def test_the_stream_sends_every_captured_frame_as_multipart_jpeg(capturing):
    server, _, _ = capturing
    kind, parts = _stream_parts(server, 3)
    assert kind == "multipart/x-mixed-replace; boundary=frame"
    assert all(part[:2] == b"\xff\xd8" for part in parts)  # JPEG SOI


def test_the_stream_without_capture_advances_on_stale_cache():
    server = CameraServer(
        bind=LOOPBACK, port=0, source=CountingSource(), preview_port=0, max_age_s=0.05
    ).start()
    try:
        assert len(_stream_parts(server, 2)[1]) == 2
    finally:
        server.stop()


def test_stop_ends_an_open_stream(capturing):
    import threading

    server, _, _ = capturing
    done = threading.Event()

    def read():
        from urllib.request import urlopen

        with urlopen(f"http://127.0.0.1:{server.preview_port}/stream.mjpeg", timeout=5) as r:
            while r.read(4096):
                pass
        done.set()

    threading.Thread(target=read, daemon=True).start()
    assert _until(lambda: server.served == 0 and server.cached_jpeg()[0] is not None)
    server.stop()
    assert done.wait(5)


def test_the_cli_captures_at_30_hz_by_default():
    from g1_camera import server

    assert server.parse_args([]).capture_hz == 30.0
    assert server.parse_args(["--capture-hz", "0"]).capture_hz == 0.0


def test_the_cli_binds_the_jetson_robot_address_by_default():
    from g1_camera import server, wire

    assert server.parse_args([]).bind == wire.DEFAULT_HOST
    assert server.parse_args(["--bind", "0.0.0.0"]).bind == "0.0.0.0"
