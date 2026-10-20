# G1 e-stop

Status: implemented on `g1-web-interface`; unit, fake and gantry tests pass (gantry 2026-10-19; G6-IMU not run).
Closes roadmap `feat/g1-finish-control-release`.

## Before this change

| Path | Where | Sets `controlStarted_=false` | Calls `finishControl_` |
|---|---|---|---|
| remote `L2+Up+Left` | `G1_bridge.cpp:130`, executor thread | yes | **no** |
| `/stop_control` | `bridge_core.cpp:510`, executor thread | yes | **no** |
| IMU trip in `lowStateHandler_` | `G1_bridge.cpp:178`, executor thread | yes | **no** |
| `checkState_` fail (state age, NaN, dq, IMU) | `bridge_core.cpp:264`, control thread | yes | yes (logs only) |
| command timeout (`tValid_`) | `bridge_core.cpp:270`, control thread | yes | yes (logs only) |

- `update_()` publishes only while `controlStarted_`, so every path ends in `/lowcmd` silence.
- The three executor-thread paths flip the flag before `update_()` looks, so `finishControl_` never runs for them.
- `controlStarted_` is a plain `bool` shared by two threads; `currentState_` is written without `mutex_`.
- `/start_control`, `/ready_position_control`, `/zero_position_control` re-arm at any time: nothing latches.
- Measured 2026-10-19: `/wirelesscontroller` has 2 publishers but sent nothing in 10 s (no key held). `/lowstate.wireless_remote` arrives at 500 Hz: bytes 0–1 `0x55 0x51`, bytes 2–3 the key mask in `WirelessKey_H1_G1` bit order.
- Measured (G0): when `/lowcmd` goes silent the motors go passive (`mode=0`) about 0.4 s later. A bridge crash drops a standing robot.

## States

| State | Publishes | `/robot_cmd` | Arming services | Left by |
|---|---|---|---|---|
| `DISARMED` | nothing | dropped | allowed | arm → `ARMED` |
| `ARMED` | tracking frames (today's path) | accepted | allowed | stop → `STOPPED`, e-stop or fault → `LATCHED` |
| `STOPPED(mode)` | stop frame, 1 kHz | dropped | allowed | arm → `ARMED`, e-stop or fault → `LATCHED` |
| `LATCHED(mode, cause)` | stop frame, 1 kHz (nothing if latched from `DISARMED`, or 1.2 s after `/lowstate` stopped) | dropped | **refused** | `/estop_reset` → `STOPPED`, or `DISARMED` if silent |

- Arming services: `/start_control`, `/ready_position_control`, `/zero_position_control`, `/start_hand_control`.
- `estop.require_remote: true`: every arming service is refused unless the remote's header (`0x55 0x51`) arrived in the last 0.5 s. Measured 2026-10-20: remote off = all 40 bytes 0; at rest the bytes stay constant, so only the header counts. Remote lost while armed: open question.
- Latched, `/lowstate` older than 1.2 s (0.2 s `state_loss` + 1 s): the latch goes silent and stays silent until reset (`LATCHED … silent`). A robot that restarts meanwhile runs its own motion service; old frames would fight it (2026-10-20).
- G1 arming is also refused while any other program publishes `/lowcmd` (a restarted robot's motion service): release it first.
- Hands: a released hand whose state is gone for 1.2 s gets no more limp frames.
- Arming moves the robot: `initControl_` ramps to the start pose over `duration` (2 s).
- One entry point `enterStop_(cause)` for every trigger, any thread. It sets the state atomically, snapshots measured q under `mutex_`, then calls `finishControl_`.
- Mode changes while stopped: an operator e-stop sets the mode it names (hold after damp re-stiffens at the current pose). A fault only turns `hold` into `damp`.
- Faults and combos fire at onset only (author, 2026-10-19): an operator hold during a lasting fault stays. Combo hold → damp (L1 added) fires; damp → hold (L1 released first) does not.
- A non-finite measured q never reaches `/lowcmd`: that joint gets damp at q 0.
- Hands: in `LATCHED` the hand timer sends its existing limp frames. `STOPPED` leaves the hands alone.
- New topic `/bridge_state` (`std_msgs/String`, transient-local, on change), e.g. `LATCHED damp remote_damp`.

## Stop frames

Built by a new `publishStopCommand_()`. It does not reuse `publishLowCommand_`: that q clamp divides by `kp`.

| Field | `damp` | `hold` |
|---|---|---|
| `q` | measured q (ignored at `kp=0`) | measured q at trigger, kept fixed |
| `dq`, `tau` | 0 | 0 |
| `kp` | 0 | joint `kp` from config |
| `kd` | `estop.damp_kd` | joint `kd` from config |
| parallel joints (ankles) | same as above | same tau conversion as `publishLowCommand_:281` |
| rate | `control_dt` (0.001 s) | `control_dt` |

- `damp` is not fully limp: no position spring, but each joint resists speed, so arms sink slowly. `kd=0` would let them drop.
- `damp_kd` default 8.0: Unitree's `create_damping_cmd` (unitree_rl_gym `deploy_real/common/command_helper.py`) sends `kp=0, kd=8` to every motor.
- `hold` keeps the torque window `|q − q_meas| ≤ tau_limit/kp`. Parameter load rejects `kp ≤ 0`.
- Re-arm from `damp` steps kp from 0 to config at the measured pose: position error 0, so torque starts at about `kd·dq`.

## Triggers

| Trigger | Cause | Mode | Latches |
|---|---|---|---|
| remote `L1+R1` | `remote_damp` | damp (fixed) | yes |
| remote `R1+R2` | `remote_hold` | hold (fixed) | yes |
| `/estop_damp` (`Trigger`) | `service_damp` | damp (fixed) | yes |
| `/estop_hold` (`Trigger`) | `service_hold` | hold (fixed) | yes |
| `/stop_control` | `stop` | hold (config) | **no** |
| IMU roll/pitch over threshold | `imu` | damp (config) | yes |
| state age > 0.2 s | `state_loss` | damp (config) | yes |
| state NaN, length, `dq` over limit | `state_invalid` | damp (config) | yes |
| command timeout | `cmd_timeout` | hold (config) | yes |

- GEAR sends `duration 0.02`, no `hold_position`: a GEAR stall over 0.22 s latches `cmd_timeout`.
- `/estop_reset` is refused while its cause is still present: state stale, IMU over threshold, NaN, or a combo still held.
- No remote reset: a reset combo can be hit while pressing buttons in a panic.

## Without a UI

- The remote is the primary e-stop: `L1+R1` damp, `R1+R2` hold.
- Services: the `ros2 service call` block in `README.md` § 4 (`/estop_damp`, `/estop_hold`, `/estop_reset`, `/stop_control`).
- The web page lives in `g1_web_commander`; the bridge needs none.

## Remote combos

Read keys from `/lowstate.wireless_remote` bytes 2–3 (little-endian `uint16`). Drop the `/wirelesscontroller` subscription and `L2+Up+Left`. Trigger on the first sample with a combo held; no debounce.

- `L1+R1` → damp, `R1+R2` → hold (author, 2026-10-19). Neither is a known firmware combo.
- `L1+R1+R2` matches both: damp wins.

| Firmware combo | Meaning |
|---|---|
| `L2+B` | damping / e-stop (fw 1.0.4) |
| `L1+A` | damping (fw 1.0.2) |
| `L2+R2` | enter debug mode |
| `L2+A` | position mode / debug-mode check |
| `L2+Up` | lock stand (inside today's `L2+Up+Left`) |
| `R2+A` | operation control |

Sources: Unitree G1 developer docs (remote control page) and third-party guides; check the lab robot's firmware version.

## Limits

- No wireless hardware e-stop for this G1 (author, 2026-10-19). The combos run through the bridge: a hung or crashed bridge ignores them. The robot's power button is the only stop outside software.

## Config — `G1_config.yaml`

```yaml
estop:
  damp_kd: 8.0
  remote_combo_damp: ["L1", "R1"]
  remote_combo_hold: ["R1", "R2"]
  require_remote: true   # arming refused without the remote
  mode:              # damp | hold; operator e-stops are fixed
    stop: hold
    imu: damp
    state_loss: damp
    state_invalid: damp
    cmd_timeout: hold
```

## Code shape

- `include/estop.hpp`, `src/estop.cpp`: no `rclcpp`. `SafetyLatch` (states, mode rules, reset rules), `buildStopFrame(...)`, `parseRemoteKeys(...)`.
- `BridgeCore`: owns a `SafetyLatch` beside `controlStarted_` (tracking, now atomic); `enterStop_`; `armControl_` gates the arming services; `checkState_` returns the fault; new services `/estop_damp`, `/estop_hold`, `/estop_reset`; virtual `publishStopCommand_()` with an empty default.
- `G1Bridge`: implements `publishStopCommand_`; remote parsing in `lowStateHandler_`, which now copies state under `mutex_`; `finishControl_` stays a log line.
- `include/joint.hpp`: `Joint` moved out of `bridge_core.hpp` so the pure code needs no ROS.
- `RobotCmdClient`: `estop_damp()`, `estop_hold()`, `estop_reset()`.
- H1, H1_2, T1: same latch and services, no stop frame (silent as before). Their own remote and IMU stop paths skip the latch, so `/bridge_state` and the latch are valid on the G1 only (author, 2026-10-19).
- Out of scope: the gate audit (task 2), H1/H1_2/T1 stop frames.

## Tests

| Level | What |
|---|---|
| `tools/test.sh --cpp`: gtest `test_estop` | frames for both modes incl. parallel joints; latch refuses arming; reset refused while cause present; operator hold after damp; fault never turns damp into hold; both combos → damp; trigger from `DISARMED` stays silent; key parsing; remote header; silence after state loss |
| `tools/test.sh --estop`: `drive_estop.py` against `fake_g1.py` | both combos, both services, `/stop_control`, timeout, IMU escalation, every refusal and reset, live on `/lowcmd` and `/bridge_state`; arming refused without the remote; `state_loss` → silent after 1.2 s. Not covered: `state_invalid` |
| `tools/test.sh --wire` | GEAR's stream ends in a `cmd_timeout` hold |
| gantry | G0–G6 below |

Gantry, robot hanging, operator on the remote:

- **G0** Arm from measured, kill the bridge, watch 10 s: do motors hold the last command or go passive?
- **G1** Arm, move to ready, `/estop_damp`: arms sink slowly; `ros2 topic hz /lowcmd` ≈ 1000.
- **G2** `/estop_hold`: pose holds; push an arm by hand, it springs back.
- **G3** After either e-stop: `/robot_cmd` ignored, `/start_control` false, `/estop_reset` true, `/start_control` true.
- **G4** `L1+R1` damps, `R1+R2` holds, `R1+R2` during damp re-stiffens; firmware does nothing else.
- **G5** `/stop_control`: holds, `/start_control` works without reset.
- **G6** Faults: one `/robot_cmd` without `hold_position` → hold after timeout; `imu_rpy_threshold: 0.05`, tilt the gantry → damp.

## Gantry results, 2026-10-19

Author on the remote, robot hanging. G6-IMU not run (needs `imu_rpy_threshold: 0.05`).

| Test | Result |
|---|---|
| G0 bridge killed while armed | motors go `mode=0` (passive) about 0.4 s after the last `/lowcmd`; soft, not frozen |
| G1 `/estop_damp` | `kp=0 kd=8` at 1 kHz; a lifted arm sinks back (peak 0.44 rad/s), shoulder yaw stays where pushed |
| G2 `/estop_hold` | stiff, springs back |
| G3 latch | `/robot_cmd` ignored (0.0000 rad), arm refused, reset to `STOPPED`, re-arm ok |
| G4 remote | `L1+R1` damp, `R1+R2` hold, damp<->hold while latched; firmware did nothing else |
| G5 `/stop_control` | `STOPPED hold stop`, re-arm without reset |
| G6 timeout | `LATCHED hold cmd_timeout` 0.70 s after one command with `duration 0.5` |
| Mid-motion | shoulder pitch lifted 0.8 rad over 8 s: `L1+R1` sinks back in ~2 s; `R1+R2` freezes, **snappy** (brake profile: `docs/psi0_gates.md` gate 10) |
