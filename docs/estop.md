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
- `estop.require_remote: true` (the G1 default when the key is missing, with a warning): every arming service is refused unless the remote's header (`0x55 0x51`) arrived in the last 0.5 s. Measured 2026-10-20: remote off = all 40 bytes 0; at rest the bytes stay constant, so only the header counts. Remote lost while armed: open question. Ready and zero count as arming even while `ARMED`: they move every joint, so they also need the remote and a free `/lowcmd`.
- Latched, `/lowstate` older than 1.2 s (0.2 s `state_loss` + 1 s): the latch goes silent and stays silent until reset (`LATCHED … silent`). A robot that restarts meanwhile runs its own motion service; old frames would fight it (2026-10-20).
- G1 arming is also refused while any other program publishes `/lowcmd` (a restarted robot's motion service): release it first. One that starts while the bridge publishes latches `damp other_lowcmd` within about 1 s (checked once a second); the damp frames go on until reset or exit.
- `/start_control`, `/ready_position_control`, `/zero_position_control` exist only after the startup waits (`/lowcmd` free, `/lowstate` up): DDS would store a call made during the wait and arm once the bridge spins.
- Hands: a released hand whose state is gone for 1.2 s gets no more limp frames.
- Arming moves the robot: `initControl_` ramps to the start pose over `duration` (2 s). It zeroes dq and tau: the last client's feedforward never rides on the arming hold, which has no timeout.
- One entry point `enterStop_(cause)` for every trigger, any thread. It sets the state atomically, snapshots measured q under `mutex_`, then calls `finishControl_`.
- Mode changes while stopped: an operator e-stop sets the mode it names (hold after damp re-stiffens at the current pose). A fault only turns `hold` into `damp`.
- Faults and combos fire at onset only (author, 2026-10-19): an operator hold during a lasting fault stays. Combo hold → damp (R2 added) fires; damp → hold (R2 released first) does not.
- A non-finite measured q never reaches `/lowcmd`: in a stop frame that joint gets damp at q 0; a tracking frame with any non-finite field is not sent and latches `state_invalid`.
- Hands: in `LATCHED` the hand timer sends its existing limp frames. `STOPPED` leaves the hands alone.
- SIGINT or SIGTERM while publishing: latches `damp shutdown` over any earlier cause, sends damp frames for 1 s (`SHUTDOWN_DAMP_S`), then exits silent. Final: no reset, re-arm or mode change. From `DISARMED` or a silent latch it exits silent at once. SIGKILL or a crash: silent at once (G0).
- New topic `/bridge_state` (`std_msgs/String`, transient-local, on change), e.g. `LATCHED damp remote_damp`.

## Robot link — `/robot_link`

`std_msgs/String`, transient-local, on change. G1 and T1. Tracks state gaps and `LowState.tick` (T1: no tick, so no RESTARTED), armed or not.

| Value | When | Cleared by |
|---|---|---|
| `OK` | a new bridge; after a clear | — |
| `LOST <s>` | no `/lowstate` for 0.2 s; again once a second | `/lowstate` back |
| `BACK <gap_s> no_restart` | back after a gap, tick moved on: a link drop | successful `/estop_reset` or arming |
| `RESTARTED <gap_s>` | back after a gap, tick not higher: a reboot | a new bridge only |

- `RESTARTED` while `ARMED`, `STOPPED` or latched: latches `damp robot_restart` (silent by then: the gap went past 1.2 s).
- `RESTARTED` while `DISARMED`: nothing latches.
- After `RESTARTED` every arming service, Ready/Zero and `/estop_reset` are refused: `robot restarted: restart the bridge to release /lowcmd`. `robot-bringup.sh` releases the motion service at start.
- The bridge never releases `/lowcmd` itself (roadmap: motion-service takeover).
- A tick drop without a 0.2 s gap is ignored: a stray sample must not latch a running robot.
- Not checked on the G1: that its tick restarts at boot (roadmap).

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
| remote `R1+R2` | `remote_damp` | damp (fixed) | yes |
| remote `L1+R1` | `remote_hold` | hold (fixed) | yes |
| `/estop_damp` (`Trigger`) | `service_damp` | damp (fixed) | yes |
| `/estop_hold` (`Trigger`) | `service_hold` | hold (fixed) | yes |
| SIGINT, SIGTERM | `shutdown` | damp (fixed) | yes, final; exits 1 s later |
| `/stop_control` | `stop` | hold (config) | **no** |
| IMU roll/pitch over threshold | `imu` | damp (config) | yes |
| state age > 0.2 s | `state_loss` | damp (config) | yes |
| state NaN, length, `dq` over limit; a non-finite tracking frame | `state_invalid` | damp (config) | yes |
| command timeout | `cmd_timeout` | hold (config) | yes |
| another `/lowcmd` publisher while publishing | `other_lowcmd` | damp (fixed) | yes |
| `/robot_link` `RESTARTED` while not `DISARMED` | `robot_restart` | damp (fixed) | yes; no reset |

- GEAR sends `duration 0.02`, no `hold_position`: a GEAR stall over 0.22 s latches `cmd_timeout`.
- `/estop_reset` is refused while its cause is still present: state stale, IMU over threshold, NaN, a combo still held, or another `/lowcmd` publisher.
- Body arming (`start_control`, `ready`/`zero` while disarmed) is refused while `checkState_` reports a fault (state stale, NaN, `dq`, IMU); the reply names the cause, e.g. `robot state fault: state_loss`.
- No remote reset: a reset combo can be hit while pressing buttons in a panic.
- State age, command timeout, remote age, the hand watchdogs and their throttled logs run on one steady clock (`steadyClock_`): a wall-clock step does not blind them. Not tested: the clock step itself.

## Without a UI

- The remote is the primary e-stop: `R1+R2` damp (one hand), `L1+R1` hold.
- Services: the `ros2 service call` block in `README.md` § 4 (`/estop_damp`, `/estop_hold`, `/estop_reset`, `/stop_control`).
- The web page lives in `g1_web_commander`; the bridge needs none.

## Remote combos

Read keys from `/lowstate.wireless_remote` bytes 2–3 (little-endian `uint16`). Drop the `/wirelesscontroller` subscription and `L2+Up+Left`. Trigger on the first sample with a combo held; no debounce.

- `R1+R2` → damp, `L1+R1` → hold (author, 2026-10-23; swapped from 2026-10-19 so one hand damps). Neither is a known firmware combo.
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
  remote_combo_damp: ["R1", "R2"]
  remote_combo_hold: ["L1", "R1"]
  require_remote: true   # arming refused without the remote
  mode:              # damp | hold; operator e-stops are fixed
    stop: hold
    imu: damp
    state_loss: damp
    state_invalid: damp
    cmd_timeout: hold
```

## Code shape

- `include/estop.hpp`, `src/estop.cpp`: no `rclcpp`. `SafetyLatch` (states, mode rules, reset rules), `stopFrame(...)`, `remoteKeys(...)`, `remoteHeader(...)`, `parseCombo(...)`.
- `BridgeCore`: owns a `SafetyLatch` beside `controlStarted_` (tracking, now atomic); `enterStop_`; `armControl_` gates the arming services (ready/zero while `ARMED` run its remote and `/lowcmd` checks themselves); each robot bridge creates the body ones with `createArmingServices_()` after its startup waits, except X2, which makes its own three there; `checkState_` returns the fault; new services `/estop_damp`, `/estop_hold`, `/estop_reset`; virtual `publishStopCommand_()` with an empty default.
- `G1Bridge`: implements `publishStopCommand_`; remote parsing in `lowStateHandler_`, which now copies state under `mutex_`; `finishControl_` stays a log line.
- `include/joint.hpp`: `Joint` moved out of `bridge_core.hpp` so the pure code needs no ROS.
- `RobotCmdClient`: `estop_damp()`, `estop_hold()`, `estop_reset()`.
- T1 (2026-10-25): as the G1: stop frames, gates, `/robot_link`, the gamepad on `/remote_controller_state` (`estop.remote_keys: t1`, damp RB+RT, hold LB+RB); custom mode on arming, damping mode at shutdown (`booster_mode.hpp`). A gamepad message within `REMOTE_TIMEOUT_S` counts as connected.
- Go2 (2026-10-25): as the G1 (Unitree remote, R1+R2 damp, L1+R1 hold): stop frames, gates, `/robot_link` with `tick`, other `/lowcmd` sender; the frame per `go2_frame.hpp` (slots 12-19 stopped). A damped Go2 sinks to the floor.
- H1, H1_2: same latch and services, no stop frame (silent as before). Their own remote and IMU stop paths skip the latch, so `/bridge_state` and the latch are valid on the G1 and T1 only (author, 2026-10-19).
- Out of scope: the gate audit (task 2), H1/H1_2 stop frames.

## Tests

| Level | What |
|---|---|
| `tools/test.sh --cpp`: gtest `test_estop` | frames for both modes incl. parallel joints; latch refuses arming; reset refused while cause present; operator hold after damp; fault never turns damp into hold; both combos → damp; trigger from `DISARMED` stays silent; key parsing; remote header; silence after state loss; shutdown damps and is final; `other_lowcmd` latches damp over hold; `robot_restart` names itself over a silent `state_loss`; `LinkWatch` values and clears |
| `tools/test.sh --link`: `drive_link.py` against `fake_g1.py` | `/fake_g1/pause` while `DISARMED` and `ARMED`: `LOST`, `BACK … no_restart`, cleared by arming and by reset; `/fake_g1/reboot` while `ARMED`: `RESTARTED`, `LATCHED damp robot_restart silent`, no frame, reset/arm/Ready refused also once `/lowcmd` is free; a fresh bridge starts `OK`, reboot while `DISARMED`: nothing latches, arming refused |
| `tools/test.sh --estop`: `drive_estop.py` against `fake_g1.py` | no `/start_control` while another node holds `/lowcmd` at start; both combos, both services, `/stop_control`, timeout, IMU escalation, every refusal and reset, live on `/lowcmd` and `/bridge_state`; arming refused without the remote; `ros2 param set` on a joint gain refused, value kept; another `/lowcmd` publisher while armed latches `damp other_lowcmd` within 1.5 s, reset refused while it is there; re-arm after a command with dq and tau: every frame has dq 0 and tau 0 (`initControl_`); `state_loss` → silent after 1.2 s; a NaN q, a joint over its `dq_limit` (0.9× stays armed) and a NaN IMU each latch `damp state_invalid` from `ARMED`, reset refused while present; then `drive_shutdown.py`: SIGINT while armed → damp for 1 s, then silent |
| `tools/test.sh --wire` | GEAR's stream ends in a `cmd_timeout` hold |
| `tools/test.sh --gates`: `drive_no_estop_block.py` | a config without `estop:` still refuses arming without the remote |
| `tools/test.sh --hands`: `drive_hand_cmd.py --check-estop` | `/estop_damp` mid-stream: both hands get limp frames (kp, kd 0) while commands still arrive; `start_hand_control` refused while latched |
| gantry | G0–G6 below |

Gantry, robot hanging, operator on the remote:

- **G0** Arm from measured, kill the bridge, watch 10 s: do motors hold the last command or go passive?
- **G1** Arm, move to ready, `/estop_damp`: arms sink slowly; `ros2 topic hz /lowcmd` ≈ 1000.
- **G2** `/estop_hold`: pose holds; push an arm by hand, it springs back.
- **G3** After either e-stop: `/robot_cmd` ignored, `/start_control` false, `/estop_reset` true, `/start_control` true.
- **G4** (2026-10-19 combos) `L1+R1` damps, `R1+R2` holds, `R1+R2` during damp re-stiffens; firmware does nothing else.
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
| G4 remote | (2026-10-19 combos) `L1+R1` damp, `R1+R2` hold, damp<->hold while latched; firmware did nothing else |
| G5 `/stop_control` | `STOPPED hold stop`, re-arm without reset |
| G6 timeout | `LATCHED hold cmd_timeout` 0.70 s after one command with `duration 0.5` |
| Mid-motion | shoulder pitch lifted 0.8 rad over 8 s: `L1+R1` sinks back in ~2 s; `R1+R2` freezes, **snappy** (brake profile: `docs/psi0_gates.md` gate 10) |

## Gantry acceptance, after PR #7 (not run)

PR #7 (gates, remote rule, silent latch) merged on 2026-10-22 without it. Bridge and g1_web_commander built in one workspace `<ws>`:

```bash
B=<ws>/src/humanoid_bridge/tools; C=<ws>/src/g1_web_commander/tools
$B/g1-bringup.sh --stop
$B/g1-bringup.sh --workspace <ws> --iface enp4s0
$C/g1-commander.sh --workspace <ws> --adapter humanoid_locoman_vla   # own tmux pane
```

| # | Do | Expect |
|---|---|---|
| 1 | read the commander start output | `bridge interface` ok, `G1 limits vs URDF` ok |
| 1b | open the Jog tab | 29 motors: the table came from `/robot_bridge` |
| 2 | remote off, click Arm | refused "remote not connected"; pill `no remote` |
| 2b | after a robot restart, before the release: Arm | refused "other /lowcmd publisher" |
| 3 | remote on, Arm at measured pose | pill `remote`; armed, nothing moves |
| 4 | `ros2 service call /start_control bridge_interface/srv/SetDefaultPosition "{default_position: [0.0, 0.0]}"` | `success: false`, nothing moves |
| 5 | armed, no jog: `$B/g1-step-test.py --joint R_ELBOW --delta 0.5` | elbow ramps, no snap; both lines PASS (<= 3 rad/s) |
| 6 | page DAMP, HOLD; remote `R1+R2` (damp), `L1+R1` (hold); RESET | as on 2026-10-19, combos swapped |
| 7 | armed, pull the Ethernet cable 5 s, plug it in | log `No /lowstate for 1.2 s: … silent`; robot stays passive after the cable is back; RESET -> `DISARMED` |
| 8 | `$B/g1-bringup.sh --stop` | `stopped`, no SIGKILL line |

- `tools/test.sh` with the robot cabled: only in a namespace with no NIC: `unshare -rn bash -c 'ip link set lo up multicast on; ip route add 224.0.0.0/4 dev lo; …; tools/test.sh --rmw cyclonedds'`.
- `ros2 topic` shows 0 publishers after a link drop: use `--no-daemon`.
