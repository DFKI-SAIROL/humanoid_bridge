# g1_sim

MuJoCo G1 in place of the robot: publishes `/lowstate` at 500 Hz, reads `/lowcmd`; only its fake motion service publishes `/lowcmd`, after a reboot.

| | |
|---|---|
| Model | mjlab `g1.xml` (29 DoF rev 1.0, rubber hands), or unitree `g1_29dof_with_hand.xml` (Dex3); `$G1_MJCF` or `--mjcf`; `ros2 param get /g1_sim mjcf` prints the loaded file |
| Added | floor, pelvis IMU sensors, motor armature, joint friction, `d435` camera on `torso_link` |
| Motors | `tau + kp(q - q_j) + kd(dq - dq_j)` per `/lowcmd` motor, clipped to its effort; mode 0 = no torque; all passive 0.4 s after the last `/lowcmd`, as the G1 (G0) |
| `LowState` | remote header `0x55 0x51`, `mode_machine` 5, IMU quaternion/rpy/gyro/accel, `tick` = sim ms |
| Dex3 | a model with both hands' joints: `/dex3/{left,right}/state` (`HandState`, 7 motors) at 100 Hz; `/dex3/{left,right}/cmd` (`HandCmd`) PD per finger, clipped to the MJCF effort; slack until the first command |
| Camera | `d435` rendered at 30 Hz, served by `g1_camera.CameraServer`: ZMQ 5556, preview `:5557/frame.jpg`, on 127.0.0.1 (`--camera-bind`); `--no-camera` |
| Viewer | MuJoCo window when `$DISPLAY` is set; in docker: `-e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro` |
| Gantry | spring on the pelvis, feet 5 cm up; on at start |
| Guard | refuses to start unless DDS stays on loopback (`CYCLONEDDS_URI` names only `lo`, or `ROS_LOCALHOST_ONLY=1`) |
| Inputs | `/g1_sim/gantry` (Bool), `/g1_sim/remote` (Bool), `/g1_sim/keys` (UInt16), `/g1_sim/reset` (Empty) |
| Link drop | `/g1_sim/link` (Float32 s): no state out, no command in; sim time and `tick` run on |
| Reboot | `/g1_sim/reboot` (Float32 s): as a link drop, then a reset, `tick` from 0, motion service `ai` on `/lowcmd` (mode-0 frames, 100 Hz) |
| Motion switcher | `CheckMode`, `SelectMode`, `ReleaseMode` on `/api/motion_switcher/{request,response}` (`unitree_api`), as unitree_sdk2py calls them; release drops the `/lowcmd` publisher. No motion service at start |

## Requirements

```bash
g1_web_commander/scripts/fetch_assets.sh     # mjlab g1.xml -> ~/.local/share/g1_web_commander/g1_mjlab
g1_web_commander/scripts/fetch_assets.sh --model unitree   # Dex3: g1_description/g1_29dof_with_hand.xml
docker build -t g1-sim:humble robot_stacks/g1/g1_sim   # ROS 2 Humble, mujoco 3.3.6, MUJOCO_GL=egl, unitree_sdk2py
```

## 1. Build

```bash
WS=~/.cache/g1_sim/ws; mkdir -p $WS/src/humanoid_bridge
docker run --rm -it --user $(id -u):$(id -g) -e HOME=/tmp \
  -e G1_MJCF=/assets/g1_mjlab/g1.xml -v ~/.local/share/g1_web_commander:/assets:ro \
  -v $WS:/ws -v $PWD:/ws/src/humanoid_bridge:ro g1-sim:humble bash
. /opt/ros/humble/setup.bash
/ws/src/humanoid_bridge/tools/build.sh --workspace /ws -- --packages-up-to robot_bridge g1_sim
```

## 2. Run, in that container

```bash
. /ws/install/setup.bash && . /ws/src/humanoid_bridge/bridge/robot_bridge/tests/integration/loopback_dds.sh
ros2 run g1_sim g1_sim &          # --no-gantry; MuJoCo window when $DISPLAY is set (--no-viewer)
/ws/src/humanoid_bridge/tools/g1-bringup.sh --sim --workspace /ws   # releases, starts G1_bridge; --foreground for a supervisor
ros2 topic pub -1 /g1_sim/gantry std_msgs/msg/Bool "{data: false}"   # drop it
```

## 3. Verify

```bash
pytest robot_stacks/g1/g1_sim/tests/     # no ROS; skips without mujoco or g1.xml
tools/test.sh --sim                      # bridge arms the sim, holds, /robot_cmd, camera, remote damp;
                                         # sim-link: link drop, reboot, restart via g1-bringup.sh --sim
G1_MJCF=~/.local/share/g1_web_commander/g1_description/g1_29dof_with_hand.xml tools/test.sh --sim   # plus: Ready opens both Dex3
robot_stacks/g1/g1_sim/tests/integration/run_sim_amo_test.sh   # AMO stands the sim 30 s, gantry off; own netns, psi0 venv + mujoco
```

## What breaks it

- **`no G1 model`** → run `fetch_assets.sh`, or set `G1_MJCF`.
- **`camera off: ...`** → no GL: `MUJOCO_GL=egl` (Mesa, no GPU needed) or `osmesa`; the image sets `egl`.
- **Camera ports in use** → a second sim, or a real/fake camera server on 5556/5557: `--camera-port`, `--preview-port`.
- **Bridge damps with "Robot signal lost"** → the sim fell behind real time (log: `behind real time`); `/lowstate` older than 0.2 s trips it.
- **Robot falls after `/g1_sim/gantry false`** → expected: the bridge holds joints, it does not balance.
- **`g1-release-lowcmd.py --iface lo` times out** → lo has no MULTICAST flag; the tool forces `multicast="true"` on lo, an older copy of it does not.
- **`g1_sim refused: CYCLONEDDS_URI binds ...`** → the shell points DDS at a NIC (`setup_unitree.sh`); source `loopback_dds.sh`.
