# G1 e-stop

Status: DESIGN, approved by the author 2026-10-19. No C++ written yet.
Closes roadmap `feat/g1-finish-control-release`.

## Today

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
- Unmeasured: what the motors do when `/lowcmd` goes silent (gantry test G0).

## States

| State | Publishes | `/robot_cmd` | Arming services | Left by |
|---|---|---|---|---|
| `DISARMED` | nothing | dropped | allowed | arm → `ARMED` |
| `ARMED` | tracking frames (today's path) | accepted | allowed | stop → `STOPPED`, e-stop or fault → `LATCHED` |
| `STOPPED(mode)` | stop frame, 1 kHz | dropped | allowed | arm → `ARMED`, e-stop or fault → `LATCHED` |
| `LATCHED(mode, cause)` | stop frame, 1 kHz (nothing if latched from `DISARMED`) | dropped | **refused** | `/estop_reset` → `STOPPED`, or `DISARMED` if silent |

- Arming services: `/start_control`, `/ready_position_control`, `/zero_position_control`, `/start_hand_control`.
- Arming moves the robot: `initControl_` ramps to the start pose over `duration` (2 s).
- One entry point `enterStop_(cause)` for every trigger, any thread. It sets the state atomically, snapshots measured q under `mutex_`, then calls `finishControl_`.
- Mode changes while stopped: an operator e-stop sets the mode it names (hold after damp re-stiffens at the current pose). A fault only turns `hold` into `damp`.
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
  mode:              # damp | hold; operator e-stops are fixed
    stop: hold
    imu: damp
    state_loss: damp
    state_invalid: damp
    cmd_timeout: hold
```

## Code shape

- `include/estop.hpp`, `src/estop.cpp`: no `rclcpp`. `SafetyLatch` (states, mode rules, reset rules), `buildStopFrame(...)`, `parseRemoteKeys(...)`.
- `BridgeCore`: owns a `SafetyLatch`; `enterStop_`; `update_()` switches on state; new services `/estop_damp`, `/estop_hold`, `/estop_reset`; virtual `publishStopCommand_()` with an empty default, so H1, H1_2 and T1 keep today's output.
- `G1Bridge`: implements `publishStopCommand_`; remote parsing moves into `lowStateHandler_`; `finishControl_` stays a log line.
- Out of scope: the gate audit (task 2), H1/H1_2/T1 stop frames.

## Tests

| Level | What |
|---|---|
| gtest `test_estop` (new, `ament_cmake_gtest`) | frames for both modes incl. parallel joints; latch refuses arming; reset refused while cause present; operator hold after damp; fault never turns damp into hold; both combos → damp; trigger from `DISARMED` stays silent; key parsing |
| fake wire (`run_fake_wire_test.sh`, `lo`) | `/estop_damp` → `/lowcmd` `kp=0 kd=8` at 1 kHz; `/estop_hold` → config gains at fixed q; `/robot_cmd` ignored; `/start_control` false until `/estop_reset` |
| gantry | G0–G6 below |

Gantry, robot hanging, operator on the remote:

- **G0** Arm from measured, kill the bridge, watch 10 s: do motors hold the last command or go passive?
- **G1** Arm, move to ready, `/estop_damp`: arms sink slowly; `ros2 topic hz /lowcmd` ≈ 1000.
- **G2** `/estop_hold`: pose holds; push an arm by hand, it springs back.
- **G3** After either e-stop: `/robot_cmd` ignored, `/start_control` false, `/estop_reset` true, `/start_control` true.
- **G4** `L1+R1` damps, `R1+R2` holds, `R1+R2` during damp re-stiffens; firmware does nothing else.
- **G5** `/stop_control`: holds, `/start_control` works without reset.
- **G6** Faults: one `/robot_cmd` without `hold_position` → hold after timeout; `imu_rpy_threshold: 0.05`, tilt the gantry → damp.
