# g1_camera

Camera server for the G1's onboard Jetson, and the client that reads it.

Self-contained on purpose: nothing here imports outside the package, so this
directory alone can go to the Jetson. No ROS, no other package in this repo.

| | |
|---|---|
| Wire | ZMQ REQ/REP, `b"get"` → `[rgb_jpeg, ir_jpeg, depth]`. Contract in `wire.py` |
| Server | `server.py`, runs on the Jetson |
| Client | `client.py`, runs on the workstation |
| Sources | `sources.py` — videohub (colour) or V4L2 (infrared only) |
| Fake | `fake.py`, synthetic frames, no camera |

## Requirements

Python ≥ 3.8, `numpy`, `pyzmq`, and OpenCV. On the Jetson OpenCV comes from
JetPack — it is deliberately not in `install_requires`, because there is no
aarch64 wheel to fall back on.

## 1. Install on the Jetson

```bash
rsync -a robot_stacks/g1/g1_camera/ unitree@192.168.123.164:~/g1_camera/
ssh unitree@192.168.123.164 'pip3 install -e ~/g1_camera'
```

On the workstation it also builds with the workspace: `colcon build
--packages-select g1_camera`.

## 2. Run the server

```bash
camera_server                          # videohub, 640x480, preview on :5557, both on 192.168.123.164
camera_server --source v4l2 --list     # which /dev/videoN is which
camera_server --width 1280 --height 720
```

Both sockets listen on `--bind` only: default `192.168.123.164`, the Jetson's
robot-network address, so the clients on `192.168.123.0/24` reach it as before
and no other interface (Wi-Fi) does. `--bind 0.0.0.0` is the old every-interface
behaviour.

## 3. Read frames

```python
from g1_camera import CameraClient

client = CameraClient(host="192.168.123.164")
rgb = client.read_rgb()  # (height, width, 3) uint8, RGB
```

## 4. Verify

```bash
pytest robot_stacks/g1/g1_camera/tests/    # 22 tests, no camera, on 127.0.0.1
fake_camera_server                          # stand-in on the wire
```

## What breaks it

- **`Cannot assign requested address` at start** → this host has no
  `192.168.123.164`, so it is not the Jetson. Pass `--bind <its address>`.

- **Frames are greyscale** → that is `/dev/video2`, the infrared node. Use
  `--source videohub`, the default, for colour.
- **Geometry looks stretched** → videohub serves 16:9. `--fit crop` (default)
  takes the centre 4:3; `--fit squash` keeps the full view and distorts it.
- **A preview left open slows a run** → raise `--max-age`. Below it the preview
  grabs its own frames and takes the source lock.
- **No depth, no IR, no exposure control** on either path. `pyrealsense2` has
  no aarch64 wheel.
- **Frame size is the camera's, not a policy's.** A caller that needs a
  particular input shape passes `--width/--height`; nothing here knows about
  any policy.
