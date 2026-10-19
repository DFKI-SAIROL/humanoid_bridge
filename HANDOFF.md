# Handoff — humanoid_bridge / g1-estop-safety

> The baton. Present tense only: where the work stands *now* and what the
> next session should do. Overwrite it each session — never append. History
> lives in git and rationale in commit bodies, not here. Before the task
> ends, promote anything durable (project status, repo instructions, commit
> body) and delete this file from the branch: a finished task hands
> nothing off, and merged, a leftover baton strays onto the default branch.

- **Repo:** humanoid_bridge
- **Branch:** `g1-estop-safety`
- **Worktree:** /home/lukasm/git/worktrees/humanoid_bridge/g1-estop-safety
- **Last updated:** 2026-10-19 15:44 CEST · local (adminsairol-MS-7E06)

## State

Not started. Two tasks, both needed before psi0 commands the real G1.

Robot, 2026-10-19: cabled to the lab box (`enp4s0`, robot `192.168.123.164`),
hanging in the gantry, debug mode (`L2+R2` done, `/lowcmd` released). Remote
paired, visible in `/lowstate.wireless_remote`. `G1_bridge` rebuilt from
`1a48b9e`, running UNARMED in tmux `g1-bridge` (unarmed = publishes nothing).
Nothing has ever been armed on the robot.

How stopping works today:
- `L2+Up+Left`, `/stop_control`, IMU trip, state loss (>0.2 s) and command
  timeout (`tFinal+0.2` unless `hold_position`) only set `controlStarted_ = false`.
- `bridge_core.cpp` `update_()` then stops calling `publishLowCommand_()`:
  `/lowcmd` goes silent and the motors keep the last command at full kp/kd.
- `G1Bridge::finishControl_()` (`G1_bridge.cpp:336`) only logs. T1 damps (`T1_bridge.cpp:304`).
- Unmeasured: what the motors do when `/lowcmd` goes silent.

Task 1, e-stop with two modes:
- `damp`: publish `kp=0`, `kd>0` from config, at 1 kHz.
- `hold`: hold the MEASURED q at trigger time, config gains.
- Latching: after a trigger `/robot_cmd` is dropped until an explicit re-arm.
- Triggers: a remote combo, a ROS service, and the existing faults (IMU,
  state loss, timeout) through `finishControl_`, mode configurable per fault.
- Closes roadmap item `feat/g1-finish-control-release`.

Task 2, audit the gates for psi0. psi0 sends a 30 Hz action chunk every 400 ms,
via GEAR (`adapters/gear_wbc`, 50 Hz `/robot_cmd`) or a planned arms-only node
(psi0 arm columns to `/robot_cmd`, legs and waist held at the measured pose).
Current gates:
- q clamped to `q_min/q_max` per joint (`G1_config.yaml`): check against Unitree's limits.
- Implicit torque window `|q - q_meas| <= tau_limit/kp` (`G1_bridge.cpp:271`); divides by zero at `kp=0`.
- kp/kd range `[0, 5000]` for both limb groups: no real gain guard.
- `dq_limit` clamps only the feedforward `dq`.
- No rate limit on target q: the client's `duration` alone sets how fast a jump runs.
- IMU trip 1.0 rad roll/pitch: loose on a gantry.
- `start_control` does not validate length (roadmap `fix/start-control-validate`).

Done when: both modes and the latch are tested on the gantry, a table
(gate, value, psi0 need, change) is in `docs/`, and its changes are merged with unit tests.

## Next action

Read `bridge_core.cpp` `update_()`, `stopControlServiceCB_`, `checkState_`
and `G1_bridge.cpp` `publishLowCommand_`, `finishControl_`, `wireless_callback`.
Write the e-stop design to `docs/estop.md`: modes, triggers, latch, re-arm,
remote combo (one Unitree firmware does not use). Get the author's approval
before writing C++.

## Blockers

- Ask the author whether the lab has a wireless hardware e-stop for this G1.

## Gotchas (unpromoted)

- `tools/test.sh` wire stages start a bridge that publishes `/lowcmd`: check
  which NIC they bind before running them with the robot on `enp4s0`.
- Stop the running bridge (`kill -- -<pid>`, pid in the `g1-bridge` pane) before
  rebuilding. `tools/build.sh --workspace ~/bridge_ws`: the clone is reached
  through a symlink, so workspace detection fails without it.
- A damping path must not reuse `publishLowCommand_`'s q clamp, which divides by `kp`.
- The installed binary is built from `~/bridge_ws/src/humanoid_bridge` -> `~/github/humanoid_bridge`,
  not this worktree: point the symlink here or build a separate workspace.
