# Bridge gates for psi0

Status: DESIGN, awaiting the author's approval. No C++ written.

Every check between a `/robot_cmd` and `/lowcmd` on the G1, against what psi0
sends. psi0 numbers: humanoid-locoman-vla `benches/g1/findings.md`, Milestone 2
(shadow run, 2026-10-19). GEAR gains: its `registry/controllers/gear-wbc/controller.toml`.
Joint limits checked against the official `g1_29dof_with_hand.urdf`.

## psi0 on the wire

| | value |
|---|---|
| Path A | psi0 30 Hz chunks of 24 steps every 400 ms -> GEAR -> `/robot_cmd` 50 Hz, `duration 0.02`, no `hold_position` |
| Path B (planned) | psi0 arm columns -> `/robot_cmd`; legs and waist held at the measured pose |
| First target - measured | mean 0.39 rad, **max 1.71 rad** (right elbow) |
| Target speed in a chunk | p95 0.39 rad/s, **max 2.2 rad/s** |
| Jump at a chunk boundary | p95 0.04 rad, max 0.13 rad |
| GEAR gains | kp 150/200/40 legs, 250 waist, 100/40/20 arms; kd <= 5 |

## Gates

| # | Gate | Today | psi0 need | Change | Must |
|---|---|---|---|---|---|
| 1 | Target q in joint range | none on the body path; only the hands clamp | every target inside the joint range | clamp each joint's target to `[q_min, q_max]` (config = URDF on all 29 joints) | **yes** |
| 2 | Target q speed | none: `duration` alone sets the speed; order 0 is a step | <= 2.2 rad/s; a 1.7 rad first target must not be a step | per-tick rate limit on the target, see below | **yes** |
| 3 | Client interpolation | `pack_robot_cmd` / `RobotCmdClient` default `interpolation_order=0.0`: a step, `duration` ignored (gantry: 0.3 rad in < 1 s) | smooth targets | default `1.0` (linear over `duration`) | no (gate 2 covers it) |
| 4 | Torque window | `\|q - q_meas\| <= (tau_limit +- kd·Δdq)/kp`; `tau_limit` = URDF effort; divides by zero at `kp=0` (±inf or NaN) | unchanged | at `kp < 1e-6` send `q = q_meas` (q has no effect at kp 0) | yes, cheap |
| 5 | Gain range | kp, kd in `[0, 5000]` for both groups | kp <= 250, kd <= 5 (GEAR); config kp <= 300 | strong (hips, knees, waist): kp `[0, 350]`, kd `[0, 10]`; others (ankles, arms): kp `[0, 150]`, kd `[0, 10]` | no |
| 6 | Speed limits | `dq_limit` clamps feedforward `dq` and trips `state_invalid` on measured `dq`; 6 joints exceed the URDF (ankles, waist roll/pitch 37 vs 30) | <= 2.2 rad/s | set those 6 to the URDF 30; keep hip roll at 20 (stricter) | no |
| 7 | IMU trip | 1.0 rad roll or pitch (57°) | standing tilt under GEAR: not measured | 0.5 rad, after the tilt is measured in the rehearsal | no |
| 8 | Command timeout | `tFinal + 0.2 s` unless `hold_position`; latches `hold` | GEAR at 50 Hz: 0.22 s; path B: each command must arrive within the last one's `duration` + 0.2 s | none | — |
| 9 | `start_control` length | wrong length: `success`, then moves to `ready_q` (legs included) | console and bringup always send 29 | refuse a wrong length; nothing moves | no |
| 10 | `hold` stop | stiff at the measured pose; gantry: "freezes, snappy" from 0.6 rad/s | — | brake profile (recorded proposal): target `q0 + v0·(t - t²/2T)`, T = 0.2 s, then `q0 + v0·T/2`; clamped to the range and a max stop distance (0.3 rad) | no |

## Gates 1 and 2: design

In `G1Bridge::publishLowCommand_`, per joint, after the interpolation and before
the torque window:

```
q_target = clamp(q_interp, q_min, q_max)                                  # gate 1
q_cmd    = q_prev + clamp(q_target - q_prev, -v_max·dt, +v_max·dt)         # gate 2
```

- `q_prev` is the last sent target (`lastCommand_`); `dt` = `control_dt` (1 ms).
- Arming seeds `q_prev` with the measured q: at boot `lastCommand_` is all zeros, and an unseeded limiter would pull every joint toward 0.
- Stop frames bypass both (they hold or damp at the measured pose) and leave `lastCommand_` at the measured pose, so a re-arm starts from there.
- `v_max` per group, new config keys `upper_limbs_q_rate_limit`, `lower_limbs_q_rate_limit` (rad/s, same grouping as the kp ranges).
- The pure part (clamp, rate step, kp-0 window) goes into a ROS-free module with gtests, as `estop.cpp` did.

What it does to psi0's first command: a 1.7 rad elbow target becomes a ramp of
1.7 / v_max s (0.57 s at 3 rad/s), on top of the node's own 2 s `--ramp-seconds`.

## Tests

| Level | What |
|---|---|
| gtest | clamp at both limits; rate step never exceeds `v_max·dt`; seeding; kp 0 gives `q_meas`, never inf or NaN |
| fake wire | a 1.7 rad step on `/robot_cmd` -> `/lowcmd` moves at <= `v_max`; an out-of-range target stops at the limit; the first arm after boot does not move toward 0 |
| gantry | the elbow step from G-mid-motion again: smooth at `v_max`, no snap |

## Decisions for the author

1. `v_max`: arms 3.0 rad/s (psi0 max 2.2 + margin). Legs and waist: 6.0 rad/s until GEAR's leg target speeds are measured in the MuJoCo rehearsal; a too-low leg limit breaks balance when standing.
2. Gates 3-7 and 9 in this task, or only the musts (1, 2, 4)?
3. Gate 10 (the brake profile): this task, or after the first commanded psi0 run?
4. `robot_client.py` maps `L1` alone to ready and `R1` alone to zero on the G1, and operators press `L1` a moment before `R1` (the e-stop). Delete those two mappings, or only add a README trap?

Doc fixes that need no decision: README and `tools/g1-bringup.sh` still say an
abort "holds the last pose at full kp/kd"; G0 measured the motors going passive
(`mode=0`) about 0.4 s after `/lowcmd` goes silent. `docs/estop.md` still lists
that as unmeasured.
