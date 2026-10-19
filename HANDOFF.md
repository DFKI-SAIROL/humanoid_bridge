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

Task 1 (e-stop): implemented per `docs/estop.md`, review findings fixed.
`tools/test.sh` passes all five stages on CycloneDDS; wire and estop also on
Fast DDS. Gantry tests G0–G6 not run: they need an operator on the remote.

Task 3 (web console, new): design interview in progress with the author.
Settled: lab PC host; LAN + token (phone); bridge stays in tmux; no browser
deadman; one controlling tab; web stack goes upstream, thesis content of
`humanoid-locoman-vla`'s lab ui does not. Round 2 (Q9–Q15: package location,
lab attach, `/robot` page, scope, three.js, token, stack) is open. After the
design: merge `feat-motor-probe` here, rename this branch `g1-web-interface`.

Task 2 (gate audit for psi0): not started. Gates to tabulate (gate, value,
psi0 need, change) in `docs/`:
- No `q_min/q_max` clamp on the body path; only the hand path clamps (`G1_bridge.cpp`, `publishOneHand_`).
- Implicit torque window `|q - q_meas| <= tau_limit/kp` in `publishLowCommand_`; divides by zero at `kp=0`.
- kp/kd range `[0, 5000]` for both limb groups: no real gain guard.
- `dq_limit` clamps only the feedforward `dq`, and trips `checkState_` on measured dq.
- No rate limit on target q: the client's `duration` alone sets how fast a jump runs.
- IMU trip 1.0 rad roll/pitch: loose on a gantry.
- `start_control` does not validate length (roadmap `fix/start-control-validate`).

Robot: cabled on `enp4s0`, hanging in the gantry, debug mode. The OLD bridge
(`~/bridge_ws`, commit `1a48b9e`) runs UNARMED in tmux `g1-bridge`. Nothing has
ever been armed. The new build is in `~/bridge_ws_estop` (src symlink → this worktree).

Done when: G0–G6 pass on the gantry, the gate table is in `docs/`, and the
changes are merged.

## Next action

Finish the web-console design interview, write it to `docs/`, then implement.
In parallel when an operator is free: gantry tests G0–G6 (`docs/estop.md`), operator on the remote:

```bash
kill -- -<pid>                       # old bridge, pid in the g1-bridge pane
set +u; source scripts/setup_unitree.sh; source ~/bridge_ws_estop/install/setup.bash
tools/g1-bringup.sh --no-release     # then --arm-from-measured --robot-is-clear
ros2 topic echo /bridge_state        # second pane
```

## Blockers

None.

## Gotchas (unpromoted)

- Run `tools/test.sh` with the robot cabled only inside a namespace with no NIC:
  `unshare -rn bash -c 'ip link set lo up multicast on; ip route add 224.0.0.0/4 dev lo; …; tools/test.sh'`.
- `ros2 topic` on the robot's topics needs `~/bridge_ws_estop/install/setup.bash`
  sourced, or it reports the message type as invalid.
- `tools/build.sh --workspace ~/bridge_ws_estop`: the clone is reached through a
  symlink, so workspace detection fails without it.
