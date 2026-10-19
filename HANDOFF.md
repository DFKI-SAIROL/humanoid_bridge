# Handoff — humanoid_bridge / g1-estop-safety

> The baton. Present tense only: where the work stands *now* and what the
> next session should do. Overwrite it each session — never append. History
> lives in git and rationale in commit bodies, not here. Before the task
> ends, promote anything durable (project status, repo instructions, commit
> body) and delete this file from the branch: a finished task hands
> nothing off, and merged, a leftover baton strays onto the default branch.

- **Repo:** humanoid_bridge
- **Branch:** `g1-web-interface` (worktree dir keeps the old name until the session ends)
- **Worktree:** /home/lukasm/git/worktrees/humanoid_bridge/g1-estop-safety
- **Last updated:** 2026-10-19 · local (adminsairol-MS-7E06)

## State

Task 1 (e-stop): implemented per `docs/estop.md`, review findings fixed.
`tools/test.sh` passes all five stages on CycloneDDS; wire and estop also on
Fast DDS. Gantry tests 2026-10-19 (other session, author on the remote), all pass
except G6-IMU (not run: needs a restart with `imu_rpy_threshold: 0.05`):

| Test | Result |
|---|---|
| G0 bridge killed while armed | motors go `mode=0` (passive) about 0.4 s after the last `/lowcmd`; soft, not frozen |
| G1 `/estop_damp` | `kp=0 kd=8` at 1 kHz; a lifted arm sinks back under gravity (peak 0.44 rad/s), shoulder yaw stays where pushed |
| G2 `/estop_hold` | stiff, springs back |
| G3 latch | `/robot_cmd` ignored (0.0000 rad), arm refused, reset to `STOPPED`, re-arm ok |
| G4 remote | `L1+R1` damp, `R1+R2` hold, damp<->hold while latched; firmware did nothing else |
| G5 `/stop_control` | `STOPPED hold stop`, re-arm without reset |
| G6 timeout | `LATCHED hold cmd_timeout` 0.70 s after one command with `duration 0.5` |
| Mid-motion | R shoulder pitch lifted 0.8 rad over 8 s (`interpolation_order 1`): `L1+R1` at -0.44 sinks back in ~2 s; `R1+R2` at -0.63 freezes, **snappy** |

Findings to act on:
- README "an abort holds the last pose at full kp/kd" is wrong: silence -> motors passive (G0). In the gantry safe; standing, a bridge crash drops the robot.
- `pack_robot_cmd` / `RobotCmdClient` default `interpolation_order=0.0`: a step, `duration` ignored (`calculateInterpolationParams_`). A 0.3 rad target jumped in <1 s. Belongs in task 2 with the rate limit.
- `robot_client.py` maps `L1` alone to ready and `R1` alone to zero position; operators press `L1` a moment before `R1` (key log). Never run the example clients with this bridge; README trap.
- Proposal (author agreed to record, not decided): soften `hold` with a brake profile. Full gains, target `q0 + v0·(t − t²/2T)` for `t<T`, then `q0 + v0·T/2`; T = 0.2 s; clamp to `q_min/q_max` and a per-joint max stop distance (e.g. 0.3 rad); filter `dq`. Rejected: gain ramp (overshoot and return), damp-then-hold (gravity sag). psi0 shadow targets reach 2.2 rad/s, first target up to 1.7 rad from measured (humanoid-locoman-vla `benches/g1/findings.md`, Milestone 2).

Task 3 (web console): implemented (`48a5010`, `docs/robot_console.md`).
`bridge/robot_console` replaces motor_probe; 250 unit tests, `tools/test.sh --console`
passes 4/4 against the real bridge + `fake_g1.py`; checked in headless Chrome at desktop
and phone size. Lab file: humanoid-locoman-vla `benches/g1/console.yaml`, branch
`feat-robot-console` (`6b2c91f`). Not yet run on the robot (roadmap "Robot console").

Task 2 (gate audit for psi0): not started. Gates to tabulate (gate, value,
psi0 need, change) in `docs/`:
- No `q_min/q_max` clamp on the body path; only the hand path clamps (`G1_bridge.cpp`, `publishOneHand_`).
- Implicit torque window `|q - q_meas| <= tau_limit/kp` in `publishLowCommand_`; divides by zero at `kp=0`.
- kp/kd range `[0, 5000]` for both limb groups: no real gain guard.
- `dq_limit` clamps only the feedforward `dq`, and trips `checkState_` on measured dq.
- No rate limit on target q: the client's `duration` alone sets how fast a jump runs.
- IMU trip 1.0 rad roll/pitch: loose on a gantry.
- `start_control` does not validate length (roadmap `fix/start-control-validate`).

Robot: cabled on `enp4s0`, hanging in the gantry, debug mode. The NEW bridge
(`~/bridge_ws_estop`, src symlink -> this worktree) runs in tmux `g1-bridge`, last
state `LATCHED damp remote_damp`.

Done when: G0–G6 pass on the gantry, the gate table is in `docs/`, and the
changes are merged.

## Next action

Task 2, design first (author's approval before C++): the gate table (gate, value,
psi0 need, change) in `docs/`, using the findings above. Must have before psi0
commands: a `q_min/q_max` clamp on the body path and a rate limit on target q.

Gantry, when an operator is free: G6-IMU (`imu_rpy_threshold: 0.05`, tilt -> damp),
then the console checks (roadmap "Robot console").

## Blockers

None.

## Gotchas (unpromoted)

- Run `tools/test.sh` with the robot cabled only inside a namespace with no NIC:
  `unshare -rn bash -c 'ip link set lo up multicast on; ip route add 224.0.0.0/4 dev lo; …; tools/test.sh'`.
- `ros2 topic` on the robot's topics needs `~/bridge_ws_estop/install/setup.bash`
  sourced, or it reports the message type as invalid.
- `tools/build.sh --workspace ~/bridge_ws_estop`: the clone is reached through a
  symlink, so workspace detection fails without it.
- The robot's bridge runs from `~/bridge_ws_estop`: build `robot_bridge` changes in
  another workspace, or tell the author before rebuilding there.
- Start `robot_console` in a foreground tmux pane, not as a script's background job:
  SIGINT is then ignored and processes stop only after the INT grace.
