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
- **Last updated:** 2026-10-19 · local (adminsairol-MS-7E06)

## State

Task 1 (e-stop): design in `docs/estop.md`, approved by the author 2026-10-19.
No C++ written.

Task 2 (gate audit for psi0): not started. Gates to tabulate (gate, value,
psi0 need, change) in `docs/`:
- No `q_min/q_max` clamp on the body path; only the hand path clamps (`G1_bridge.cpp:553`).
- Implicit torque window `|q - q_meas| <= tau_limit/kp` (`G1_bridge.cpp:271`); divides by zero at `kp=0`.
- kp/kd range `[0, 5000]` for both limb groups: no real gain guard.
- `dq_limit` clamps only the feedforward `dq`, and trips `checkState_` on measured dq.
- No rate limit on target q: the client's `duration` alone sets how fast a jump runs.
- IMU trip 1.0 rad roll/pitch: loose on a gantry.
- `start_control` does not validate length (roadmap `fix/start-control-validate`).

psi0 path: 30 Hz action chunk every 400 ms, via GEAR (`adapters/gear_wbc`,
50 Hz `/robot_cmd`) or a planned arms-only node (legs and waist held at the measured pose).

Robot: cabled on `enp4s0`, hanging in the gantry, debug mode. `G1_bridge` from
`1a48b9e` runs UNARMED in tmux `g1-bridge`. Nothing has ever been armed.

Done when: both modes and the latch pass gantry tests G0–G6 (`docs/estop.md`),
the gate table is in `docs/`, and the changes are merged with unit tests.

## Next action

Write `estop.hpp/.cpp` and the gtest target first, then wire
`BridgeCore`/`G1Bridge`, then the fake wire stage.

## Blockers

None.

## Gotchas (unpromoted)

- `/wirelesscontroller` sends nothing with no key held; read keys from
  `/lowstate.wireless_remote` bytes 2–3. Read-only checks need `~/bridge_ws/install/setup.bash`
  sourced, or `ros2 topic` reports the message type as invalid.
- `tools/test.sh` says it pins CycloneDDS to `lo`: confirm for the Fast DDS stage
  before running it with the robot on `enp4s0`.
- Stop the running bridge (`kill -- -<pid>`, pid in the `g1-bridge` pane) before
  rebuilding. `tools/build.sh --workspace ~/bridge_ws`: the clone is reached
  through a symlink, so workspace detection fails without it.
- The installed binary is built from `~/bridge_ws/src/humanoid_bridge` -> `~/github/humanoid_bridge`,
  not this worktree: point the symlink here or build a separate workspace.
