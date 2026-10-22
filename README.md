# humanoid_bridge

ROS 2 bridge between a control client and Unitree (G1, H1, H1_2), Booster (T1)
or AgiBot (X2) humanoids. It takes commands on `/robot_cmd`, interpolates them, runs safety
checks on command and state, and publishes motor commands at 1 kHz.

```
bridge/           the bridge and its Python client, upstream-bound
  bridge_interface/   messages and services
  robot_bridge/       C++ bridge, robot_bridge/ Python client, adapters
    examples/           reference clients per robot (not built or installed)
robot_stacks/     per-robot deploy layers
  g1/                 g1_stack, g1_camera
  t1/                 placeholder; the T1 stack has not been written
scripts/          environment setup, sourced not run
tools/            helpers, run not sourced
thirdparty/       vendored unitree_ros2, booster_ros2_interface and aimdk_msgs (X2)
```

`bridge/` depends on nothing under `robot_stacks/`. That one-way rule is what
lets either layer be split out with `git subtree split --prefix=<layer>`.

Built targets are `G1_bridge`, `H1_bridge`, `H1_2_bridge`, `T1_bridge`,
`X2_bridge` (only with `-DBUILD_AGIBOT_X2=ON`) and `test_node`. Other sources under `src/` are upstream leftovers that no target
builds.

## Fork or upstream

Forked from [DFKI-SAIROL/humanoid_bridge](https://github.com/DFKI-SAIROL/humanoid_bridge).
`bridge/` is upstream-bound; `robot_stacks/` is this project's own.

| Change | Side | State |
|---|---|---|
| `/lowstate` copy bounded to `numJoint_` (upstream writes 35 into a 29-vector) | upstream | fixed here, no PR yet |
| `cmd_client.py` and `adapters/` (GEAR WBC, GEAR hands) | upstream | added here, no PR yet |
| Dex3 hand path: `HandCmd`, the 100 Hz loop, per-side guards | upstream | added here, no PR yet |
| `checkMotorCmd_` using `std::abs` (upstream's bare `abs` truncated \|v\| < 1 to 0) | upstream | fixed here, behaviour change on the body path |
| Config keymap headers advertising keys the source comments out | upstream | fixed here, no PR yet |
| `start_control` answers `success: true` on a wrong length, then moves to `ready_q_` | upstream | fixed here: empty = `ready_q_`, any other wrong length refused |
| Arming services exist during the startup waits, so a call made then arms later | upstream | fixed here: created after the waits, no PR yet |
| G1 `_default_pos` is 27 long while kp/kd are 29 | upstream | **open**, `fix/g1-default-pos-29` |
| `finishControl_` only logs on G1, H1, H1_2, so an abort holds at full gains | upstream | G1: fixed here by the e-stop latch (`docs/estop.md`); H1, H1_2: open |
| G1 tracking frame carries a NaN `/lowstate` that lands after `checkState_` (kp-0 and parallel joints) | upstream | fixed here: not sent, latches `state_invalid`, no PR yet |
| G1 target gates: range clamp, speed limit, kp-0 torque window (`docs/psi0_gates.md`) | upstream | added here, no PR yet |
| `start_control` keeps the last `/robot_cmd`'s dq and tau in its endless hold | upstream | G1: fixed here, no PR yet; H1, H1_2, T1, X2: open |
| G1 arming needs the remote (`estop.require_remote`) | upstream | added here, no PR yet |
| G1 `initControl_` resets the `/lowstate` age, so arming on a stalled state tracks it for 0.2 s | upstream | fixed here: arming refused on a state fault, no PR yet |
| Two-layer split, `robot_bridge_py` -> `robot_bridge` rename | fork | this project's structure |
| `robot_stacks/g1/` (`g1_stack`, `g1_camera`) | fork | whether it stays here is DFKI's call |
| `tools/` | fork | build, test and bring-up helpers |

Details and done-when per item: `PROJECT_ROADMAP.md`.

## Requirements

| | |
|---|---|
| OS, ROS | Ubuntu 22.04, ROS 2 Humble. Foxy + Python 3.8 on-board |
| Python | 3.10 on the workstation, 3.8 on-board |
| Not pulled by rosdep | `rosidl_generator_dds_idl`, `rmw_cyclonedds_cpp`, and the Python deps (`numpy`, `pyzmq`, `msgpack`, `msgpack-numpy`, OpenCV, `pytest`, `ruff`). `tools/install-build-deps.sh` installs the lot |
| G1 / H1 | [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python) |
| T1 | [booster_robotics_sdk](https://github.com/DFKI-SAIROL/booster_robotics_sdk), built per its README. Only needed with `--t1` |

`rosidl_generator_dds_idl` is required by the vendored `unitree_*` packages and
declared in none of their `package.xml`, so nothing installs it for you.

CycloneDDS is not optional wherever GEAR is in the path: its simulator forces
raw CycloneDDS, so every ROS 2 node has to match it.

## 1. Build

Clone into a colcon workspace's `src/`, then:

```bash
source /opt/ros/humble/setup.bash
tools/install-build-deps.sh     # first time on a fresh host
tools/build.sh                  # G1, H1, H1_2
tools/build.sh --t1             # also the Booster T1
# AgiBot X2, into install_x2/ for scripts/setup_x2.sh:
tools/build.sh -- --build-base build_x2 --install-base install_x2 \
    --cmake-args -DBUILD_BOOSTER_T1=OFF -DBUILD_AGIBOT_X2=ON
```

The Python client installs with the package, so there is no `pip install` step
and every Python change needs a rebuild. `tools/build.sh --clean <pkg>` drops a
stale package tree.

## 2. Launch the bridge

`BRIDGE_IFACE` is the NIC CycloneDDS binds, default `enp4s0`. Set it to match
your cabling; a wrong one fails silently.

```bash
BRIDGE_IFACE=<nic> source scripts/setup_unitree.sh   # G1, H1, H1_2
source scripts/setup_booster.sh                      # T1
source scripts/setup_x2.sh                           # X2 (Fast DDS)
```

On the G1, one script does the whole ordering: release `/lowcmd`, start the
bridge, wait for it, arm.

```bash
tools/g1-bringup.sh --dry-run                        # print the plan, run nothing
tools/g1-bringup.sh --iface enp4s0                   # release + bridge, no arming
tools/g1-bringup.sh --arm-from-measured --robot-is-clear
tools/g1-bringup.sh --stop                           # every G1_bridge; SIGKILL if SIGINT hangs
```

It sources ROS and `setup_unitree.sh` itself. The release runs on
`$UNITREE_PYTHON` (default: the lab venv), which needs `unitree_sdk2py`.

**Arming moves the robot**, so it needs an `--arm-*` flag and
`--robot-is-clear`. `--arm-from-measured` arms to the pose the robot already
holds, which is the smooth choice.

By hand, or for the other robots:

```bash
ros2 run robot_bridge G1_bridge --ros-args \
    --params-file bridge/robot_bridge/params/G1_config.yaml
```

Substitute `H1_bridge`, `H1_2_bridge` or `T1_bridge` and the matching config.
The bridge starts in damping mode, and refuses to start while anything else
publishes `/lowcmd`. `tools/g1-release-lowcmd.py --iface <nic>` frees it;
`L2 + R2` on the controller does the same. On G1, H1 and H1_2,
`/start_control`, `/ready_position_control` and `/zero_position_control` do not
exist until then.

## 3. Launch a client

```bash
source scripts/setup_unitree.sh
EX=bridge/robot_bridge/examples/g1
pip install -r $EX/requirements.txt      # first time
python $EX/example.py
```

The examples read the robot's own joystick. They are reference code: no
CMakeLists builds or installs them.

## 4. Operate

Between start and policy inference, lower the robot until its feet touch the
ground.

Only the keys below reach the bridge. Everything commented out in the source is
listed as "service only", so use the service call instead.

| | Start control | Stop control | Ready position | Zero position |
|---|---|---|---|---|
| G1 | service only | service only (holds) | service only | service only |
| H1 | `L2 + Start` | `L2 + Up + Left` | `L1` | `R1` |
| H1_2 | service only | `L2 + Up + Left` | service only | service only |
| T1 | `LT + RT + Start` | `Back` | `LB` | `RB` |

T1 also shuts the node down on `LT + Back`. `R2 + A` starts Unitree's own
policy, not anything here.

G1 e-stop, latched until `/estop_reset` (`docs/estop.md`): `L1 + R1` damp,
`R1 + R2` hold. Do not use `L2 + Up`: it is the firmware's lock-stand combo.

The bridge has no UI. The G1 web page (e-stop, arm, processes, jog, camera) is
[`SiweiJu/g1_web_commander`](https://github.com/SiweiJu/g1_web_commander), built in the same workspace.

```bash
ros2 service call /start_control bridge_interface/srv/SetDefaultPosition \
    "{default_position: [...]}"                 # your own initial pose
ros2 service call /stop_control std_srvs/srv/Trigger {}
ros2 service call /ready_position_control std_srvs/srv/Trigger {}
ros2 service call /zero_position_control std_srvs/srv/Trigger {}
ros2 service call /estop_damp std_srvs/srv/Trigger {}        # G1: latched, kp 0
ros2 service call /estop_hold std_srvs/srv/Trigger {}        # G1: latched, holds the pose
ros2 service call /estop_reset std_srvs/srv/Trigger {}       # refused while the cause lasts
ros2 topic echo /bridge_state                                # G1 only
```

### G1 Dex3 hands

A separate path with its own enable, so hands can run with the body idle.
`docs/hands_path.md` has the whole picture.

```bash
ros2 service call /start_hand_control std_srvs/srv/Trigger {}
ros2 topic pub /hand_cmd/left bridge_interface/msg/HandCmd ...   # 7 motors, DDS order
ros2 service call /stop_hand_control std_srvs/srv/Trigger {}
```

## 5. Verify

No hardware. Stages that cannot run are skipped with the reason.

```bash
tools/test.sh                       # unit, then body wire, then hands
tools/test.sh --unit                # needs no ROS
tools/test.sh --stale-after 3.5     # per-side hand release
```

The unit count depends on the environment: without a sourced workspace the
tests that need `bridge_interface` skip. `tools/test.sh` prints what it ran.

## What breaks it

- **G1 and H1 need `L2 + R2` first**, to enter debug mode. Nothing responds otherwise.
- **`PYTHONPATH=.` after sourcing ROS drops rclpy.** Append instead: `.:$PYTHONPATH`.
- **Clients import the *installed* client**, so rebuild after every Python change.
- **Moving a package leaves colcon's cache pointing at the old path.**
  `tools/build.sh --clean <pkg>`.
- **`start_control` with an empty `default_position` arms to `ready_q_`**, legs
  included. Any length other than 29 is refused, and nothing moves.
- **G1: no arming without the remote.** `estop.require_remote` refuses every
  arming service until the remote's header arrives: switch the remote on.
- **The first `/robot_cmd` cuts the arming ramp short.** `controlStarted_` is
  set before the 2 s ramp, so a client already streaming jumps to its own
  target at the gate-2 speed limit (G1: arms 3, rest 6 rad/s). Arm before the
  client streams.
- **Control stops 200 ms after the last command's interpolation ends**, unless
  `hold_position`. On the G1 that latches `hold` until `/estop_reset`. A chunked client must cover the gap with `duration`.
- **G1: a fault latches the mode `estop:` names** (damp for IMU, state loss,
  invalid state; hold for a command timeout). A stopped bridge leaves `/lowcmd`
  silent, and the motors go passive about 0.4 s later (`docs/estop.md`, G0).
- **H1, H1_2: an abort does not release the joints.** `finishControl_` only
  logs, so the robot holds its last pose at full kp/kd. Only T1 drops to damping.
- **`kp = 0` makes the torque clamp a no-op for that joint** (`upper_limbs_kp_min: 0.0`).
- **`robot_bridge/package.xml` depends on `booster_interface` and `aimdk_msgs`
  unconditionally**, so both build even with T1 and X2 off.
- **X2: never run on a robot in this fork.** Every stop damps and latches in
  `X2Bridge`; motion again needs a bridge restart, not `/estop_reset`.
- **Hand control needs its own `/start_hand_control`.** `/start_control` does
  not enable it, and it refuses any hand whose state topic is silent.
- **`create domain error` from a DDS client**: `unitree_sdk2py` hardcodes
  `/tmp/cdds.LOG`, owned by whoever ran first. `CYCLONEDDS_URI` cannot override
  it; the inline config wins.

Limits, gains and joint tables live in `bridge/robot_bridge/params/`.
