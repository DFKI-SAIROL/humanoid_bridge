# Bridge gates for psi0

Status: decided with the author and built, 2026-10-20; unit and fake tests pass (`tools/test.sh --cpp --gates --estop`). Gantry not run.

Every check between a `/robot_cmd` and `/lowcmd` on the G1, against what psi0
sends. psi0 numbers: humanoid-locoman-vla `benches/g1/findings.md`, Milestone 2
(shadow run, 2026-10-19). GEAR gains: its `registry/controllers/gear-wbc/controller.toml`.
Joint limits checked against the official `g1_29dof_with_hand.urdf`.

## Rule

- The bridge holds one **robot envelope**, the same for every model: values come from the robot and the lab, never from a model.
- Model-specific shaping lives in the model's adapter node (`psi0_bridge_loop`, `gear_controller`). psi0: a 0.12 rad leash on the arms, as in psi0's own `robot_body.py` (`30 rad/s × 4 ms` from the measured q).
- Later, if a second model needs it: optional tighten-only limits per `RobotCmd`.

## psi0 on the wire

| | value |
|---|---|
| Path A | psi0 30 Hz chunks of 24 steps every 400 ms -> GEAR -> `/robot_cmd` 50 Hz, `duration 0.02`, no `hold_position` |
| Path B | `psi0_bridge_loop`: psi0 arm columns -> `/robot_cmd` 30 Hz, `interpolation_order 1`; legs and waist held at the measured pose |
| First target - measured | mean 0.39 rad, **max 1.71 rad** (right elbow) |
| Target speed in a chunk | p95 0.39 rad/s, **max 2.2 rad/s** |
| Jump at a chunk boundary | p95 0.04 rad, max 0.13 rad |
| GEAR gains | kp 150/200/40 legs, 250 waist, 100/40/20 arms; kd <= 5 |
| psi0's own G1 gains | kp 200 strong, 100 others, 30 wrists; kd <= 7.5 |

No current check blocks psi0. Missing: gates 1, 2, 4. Too loose: gates 5, 6, 7, 9.

## Gates

| # | Gate | Today | Decision (2026-10-20) |
|---|---|---|---|
| 1 | Target q in joint range | none on the body path | clamp to `[q_min, q_max]`; warn max 1/s. Gate 4 runs after it and can move q past a limit (see 4) |
| 2 | Target q speed | none: a step goes out as a step | rate limit per tick: `q_rate_limit` arms 3.0 rad/s, `q_rate_limit_default` 6.0; leg value revisited after the MuJoCo rehearsal |
| 3 | Client interpolation | `pack_robot_cmd` / `RobotCmdClient` default 0 (step) | default 1; `gear_controller` passes 0 explicitly (GEAR's own timing) |
| 4 | Torque window | divides by kp: at kp 0 with kd·Δdq > `tau_limit`, q = ±inf | at kp < 1e-6 send `q = q_meas`; the command's `tau` counts against `tau_limit` (not on parallel joints, which drop it). A `tau` away from a limit can move q past it; the net torque still points away (L_SHOULDER_PITCH: q_meas 2.66, dq_meas 2, tau −25 → q ≥ 2.70 > q_max 2.6704) |
| 5 | Gain range | kp, kd in `[0, 5000]` | strong (hips, knees, waist): kp `[0, 350]`; others (ankles, arms): kp `[0, 150]`; kd `[0, 10]` both |
| 6 | Speed limits | ankles, waist roll/pitch at 37 vs URDF 30 | set those 6 to 30; hip roll stays 20 (rev 1.0 URDF value) |
| 7 | IMU trip | 1.0 rad roll or pitch | keep; ~0.5 rad after the rehearsal measures standing tilt under GEAR and psi0 |
| 8 | Command timeout | `tFinal + 0.2 s`, latches `hold` | no change |
| 9 | `start_control` length | wrong length: `success`, moves to `ready_q` | empty list = `ready_q` (documented); a wrong length, a non-finite value, or one over 0.1 rad outside its range refused, nothing moves |
| 10 | `hold` stop | stiff at the measured pose ("snappy") | keep; brake profile not planned |

Not gated: self-collision. The joint ranges let an arm reach the head.

## Gates 1 and 2: design

In `G1Bridge::publishLowCommand_`, per joint, after the interpolation and before
the torque window:

```
q_target = clamp(q_interp, q_min, q_max)                                  # gate 1
q_cmd    = q_prev + clamp(q_target - q_prev, -v_max·dt, +v_max·dt)         # gate 2
```

- `q_prev` is the last sent target (`lastCommand_`); `dt` = `control_dt` (1 ms).
- Arming seeds `q_prev` with the measured q: at boot `lastCommand_` is all zeros, and an unseeded limiter pulls every joint toward 0.
- Stop frames bypass both and leave `lastCommand_` at the measured pose.
- `v_max` per joint: optional `<joint>.q_rate_limit`, else `q_rate_limit_default` (absent: 0 = off, warn). Per joint, because the gain groups put the ankles with the arms.

## Also decided

| Item | Decision |
|---|---|
| `robot_client.py` remote `L1` alone (ready), `R1` alone (zero) | delete both and their help lines: they share keys with the e-stop combos and arm the bridge |
| Latched after `state_loss` | send DAMP for 1 s more, then silent until `/estop_reset`: a robot that restarts must not get old frames |
| `/robot_cmd` header | dropped when `interpolation_order` is outside [0, 1] or `duration` is NaN or negative; an order outside [0, 1] used to end the bridge process (review, 2026-10-20) |
| Config vs URDF | `g1_web_commander`'s `tools/g1-commander.sh` runs `tools/check-g1-limits.py`: it checks `G1_config.yaml` body limits against the model URDF (ranges) and `g1_29dof_with_hand.urdf` (all four, if present); listed exceptions only |

## Tests

| Level | What |
|---|---|
| gtest | clamp at both limits; every full rate tick is exactly `v_max·dt` (3 and 6 rad/s, both ways); seeding; kp 0 gives `q_meas`, never inf or NaN; the kd bias `kd·(dq_meas - dq_cmd)` shifts the window; a non-finite field fails the last frame check |
| fake wire (`--gates`, `--estop`) | a 1 rad step ramps at `v_max` within 5 %, speed and time from the frame count at `control_dt` (elbow 0.333 s at 3.0, hip 0.167 s at 6.0); a target past `q_max` stops at `q_max`; with `fake_g1` frozen (lagging), a 1 rad step stops `tau_limit/kp` from the measured q (elbow 0.5 rad); the first arm after boot holds the measured pose; `start_control` with 14 values refused, `[]` arms to `ready_q` |
| gantry | the elbow step from G-mid-motion again: smooth at `v_max`, no snap |
