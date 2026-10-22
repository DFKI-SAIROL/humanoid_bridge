# Dex3 hands through the bridge

G1 Dex3-1 hand control, beside the body path rather than inside it.
`num_joint` stays 29 — the hands are a separate device, on their own state
topic, at their own rate.

## Wire

| | |
|---|---|
| in | `/hand_cmd/left`, `/hand_cmd/right` — `bridge_interface/HandCmd`, 7 motors |
| out | `/dex3/left/cmd`, `/dex3/right/cmd` — `unitree_hg/HandCmd` |
| state in | `/dex3/left/state`, `/dex3/right/state` — `unitree_hg/HandState` |
| enable | `/start_hand_control`, `/stop_hand_control` — `Trigger`, both hands |
| rate | `hand_control_dt`, 0.01 s, its own wall timer |

`unitree_hg` already ships `HandCmd.msg` and `HandState.msg`, so the C++ bridge
owns the hands natively over ROS 2 — no `unitree_sdk2py`, no new vendoring.

```
bridge_interface/msg/HandCmd.msg
  float64 duration
  bool hold_position
  MotorCmd[] motor_cmd    # 7, DDS order
```

No `interpolation_order`: hands always ramp linearly from the last published
frame. The body's order-0 low-pass buys nothing at 100 Hz.

Hand state is consumed by the bridge for its own guards and not republished.
`g1_stack` keeps reading `rt/dex3/*/state` on raw DDS; both subscribers coexist.

## Joint order

Two orderings disagree, inverted by group:

| Interface | Order |
|---|---|
| `/hand_cmd/*`, `rt/dex3/*`, GEAR's `HandCommandSender` | `thumb_0 thumb_1 thumb_2 middle_0 middle_1 index_0 index_1` |
| GEAR's 31-slot goal array, `g1_stack.gear.goal` | `index_0 index_1 middle_0 middle_1 thumb_0 thumb_1 thumb_2` |

`/hand_cmd/*` is DDS order. `HandCommandSender.send_command` is DDS order too,
so `adapters/gear_hands.py` is a pass-through and remapping there would be the
bug. Callers holding a goal-order pose use
`g1_stack.gear.goal.hand_from_pose(pose, side)`, which resolves by joint name.

Ranges are mirrored per side: the left hand closes negative, the right positive.
Per-joint config blocks carry that; one shared table cannot.

## Config — `G1_config.yaml`

Limits come from the Dex3-1 URDFs, `psi0/real/assets/unitree_hand/
unitree_dex3_{left,right}.urdf`: `q_min`/`q_max` from `<limit lower/upper>`,
`dq_limit` from `velocity` (6.857 on `thumb_0`, 12 elsewhere), `tau_limit` from
`effort` (2.45 on `thumb_0`, 1.4 elsewhere).

```yaml
num_hand_joint: 7
hand_control_dt: 0.01
hand_state_timeout: 0.2
hand_temperature_limit: 80.0
hand_kp_min/max: 0.0 / 10.0
hand_kd_min/max: 0.0 / 2.0
hand_ready_q: [0.0 x7]        # zero is the open, extended end on both hands
left_hand_joint_names:  [L_HAND_THUMB_0, ...]   # 7 blocks, idx 0-6
right_hand_joint_names: [R_HAND_THUMB_0, ...]   # 7 blocks, idx 0-6
```

A config without the `hand_*` keys logs one warning and leaves the body path
untouched, so older configs and the H1 still work.

## Guards

| | Body | Hand |
|---|---|---|
| state freshness | 0.2 s, latches `damp state_loss` | per side, releases that hand |
| state NaN/Inf, `dq` limit | latches `damp state_invalid` | yes, plus a temperature ceiling |
| cmd size | 29 or reject | 7 or reject |
| cmd NaN/Inf, absurd magnitude | reject | reject |
| `kp`/`kd` | clamped by strong/weak class | clamped by `hand_kp_*`/`hand_kd_*` |
| `q` | torque-implied bound in `publishLowCommand_` | clamped to `[q_min, q_max]` |
| watchdog | `duration + 0.2 s`, then latches `hold cmd_timeout` | `duration + 0.2 s`, then limp |

`BridgeCore::checkMotorCmd_` is the shared per-motor half; both loops call it.

Body modes per cause: `estop.mode` in `G1_config.yaml` (`docs/estop.md`). A hand
release is `kp = kd = 0`, unlike the body's `hold` on a command timeout: a hand
here holds nothing critical, and limp cannot cook a stalled finger.

A released hand keeps getting explicit limp frames. `stop_hand_control` instead
goes silent and lets the mode's timeout bit release the fingers.

## Mode bits

Packed bridge-side, once, per motor: `(i & 0x0F) | (1 << 4) | (1 << 7)` — id,
status enable, timeout enable. A motor left at mode 0 ignores the command, so
all 7 are written every frame. `bridge_interface/MotorCmd.mode` from a client is
overwritten.

## Python

```python
client.start_hand_control()                       # both hands, independent of start_control
client.send_hand_cmd("left", q, kp=..., kd=...)   # 7 long, DDS order
client.stop_hand_control()
```

On both `RobotCmdClient` and `RobotClient`. There are no baked-in hand gains —
pass them, or set them once with `set_default_hand_cmd()`. `start_hand_control`
answers `success: false` when no hand is publishing state, and names any hand it
skipped. Like body arming, it is refused on a body state fault (`checkState_`:
IMU tilt past `imu_rpy_threshold`, stale or invalid `/lowstate`); the message
names the cause, e.g. `robot state fault: imu`.

`adapters/gear_hands.py` replaces GEAR's `HandCommandSender`; it does carry
GEAR's own gains, because it is a drop-in and its caller passes none.

## Verify

Hands are not mounted, so all of this is verified against a fake only.

```bash
tools/test.sh --hands               # plain, hands-stale, hands-hot runs
tools/test.sh --hands --stale-after 3.9 --hot-after 3.2
```

The hand stage runs three times: plain; the left hand's state stops at
`--stale-after`; it reports over temperature at `--hot-after` (both default
3.5 s). Both count from the first `/dex3/left/cmd`, which is
`start_hand_control`, so 3.0 to 4.0 lands mid-stream.

`fake_dex3.py` publishes state at 100 Hz and records `/dex3/*/cmd`;
`drive_hand_cmd.py` checks that `start_hand_control` is refused while another
node publishes `/lowcmd` and while the body is tilted, drives both hands through the adapter and then sends a
deliberately bad command; `check_hand_cmd.py` asserts the mode bits, the
left/right split, the ramp, the clamps, that NaN commands leave the last good
target in force, and that the body stayed idle (a `/lowcmd` probe proves the
fake records). The plain run then re-arms both hands and checks that
`/estop_damp` mid-stream sends both limp frames and refuses `start_hand_control`.
The stale and hot runs check that only the left hand releases; a hot hand keeps
getting limp frames.

## What breaks it

- **`checkExternalPublisher_` will not see `dex3_probe --command`.** It queries
  the ROS graph; `unitree_sdk2py` publishers are not ROS 2 nodes. The
  two-publisher guard is partial on the hand topics, and warns rather than
  blocking so a contested hand topic cannot stop the body bridge.
- **`unitree_hg/HandCmd.motor_cmd` is an unbounded sequence**, unlike
  `LowCmd`'s fixed `[35]`. It must be resized to 7 before indexing, or nothing
  is published.
- **Each hand has its own watchdog.** After `start_hand_control` a hand holds
  its ready pose until its first command; once its commands stop it goes limp
  `duration + 0.2 s` later, unless `hold_position`. Commanding one hand leaves
  the other alone.
- **Every run ends with both hands limp.** The watchdog fires once the driver
  stops, so a recording's last frame has zero gains — assert against the last
  *commanded* frame, not the last frame.
- **`q = 0` sits exactly on a limit** for several joints (left `thumb_2` is
  `0.0 .. 1.74532925`), so the open-hand ready pose clamps on any negative
  overshoot.
- **Hand callbacks and the hand timer all run on the executor thread.** A
  single-threaded `spin` serialises them, which is why the hand path takes no
  lock. Moving `g1_node.cpp` to a multi-threaded executor breaks that.
