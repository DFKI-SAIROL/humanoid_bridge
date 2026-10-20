# Handoff — humanoid_bridge / g1-web-interface

> The baton. Present tense only: where the work stands *now* and what the
> next session should do. Overwrite it each session — never append. History
> lives in git and rationale in commit bodies, not here. Before the task
> ends, promote anything durable (project status, repo instructions, commit
> body) and delete this file from the branch: a finished task hands
> nothing off, and merged, a leftover baton strays onto the default branch.

- **Repo:** humanoid_bridge
- **Branch:** `g1-web-interface` (worktree dir still `g1-estop-safety`; see Gotchas)
- **Worktree:** /home/lukasm/git/worktrees/humanoid_bridge/g1-estop-safety
- **Last updated:** 2026-10-19 · local (adminsairol-MS-7E06)

## State

- **E-stop (task 1):** done. Gantry passes G0–G6 except G6-IMU (`docs/estop.md`).
- **Robot console (task 3):** built (`docs/robot_console.md`). Today's fixes: smoother jog (slew at max speed 0.2/0.5/1.0 rad/s, 50 Hz) and a slider with measured/sent markers (`55136a0`). The author is testing it on the robot; results are not recorded yet. Lab file: humanoid-locoman-vla `benches/g1/console.yaml`, branch `feat-robot-console`.
- **Console 3D model (uncommitted, 2026-10-20):** default is now mjlab's G1 (`6fc2a60`, 7-capsule feet), converted by `scripts/mjcf_to_urdf.py`; the 3D view's corner tag shows the `SOURCE`. The lab `console.yaml` points at `g1_mjlab/g1.urdf` (uncommitted in humanoid-locoman-vla). The running console needs `colcon build --packages-select robot_console` in `~/bridge_ws_estop` and a restart.
- **psi0 gates (task 2):** design and findings in `docs/psi0_gates.md`. The author said to close the clamping (gates 1, 2, 4). WIP `ef366d2`: pure `gates.hpp/.cpp` and 8 passing gtests, **not called from the bridge yet**.
- **Robot:** in the gantry, debug mode, on `enp4s0`. The bridge runs from `~/bridge_ws_estop` in tmux `g1-bridge`. The console may still run in tmux `robot-console`.

## Next action (tomorrow, in order)

1. **Finish gates 1, 2, 4** (build in `~/bridge_ws_gates`):
   - `BridgeCore::loadParameters_`: optional `<joint>.q_rate_limit`, default `q_rate_limit_default` (absent: 0 = off, warn).
   - `G1Bridge::publishLowCommand_`: after the interpolation, `cmd.q = gates::limitTarget(cmd.q, last_cmd.q, joint_info, controlDt_)`; replace the inline window with `gates::torqueWindow(...)`.
   - `G1Bridge::initControl_`: seed `lastCommand_.q` with the measured q under `mutex_`, **before** `calculateInterpolationParams_(0.0, …)`. Without it, the first arm after boot pulls every joint toward 0.
   - `G1_config.yaml`: `q_rate_limit_default: 6.0`; `q_rate_limit: 3.0` on the 14 arm joints. Confirm these values with the author (decision 1).
   - Fake wire stage `gates`: a 1.7 rad step ramps at <= 3 rad/s; an out-of-range target stops at the limit; the first arm after boot does not move toward 0 (add `fake_g1.py --start-q`). `tools/test.sh --cpp` also runs `test_gates`.
   - Gantry: repeat the mid-motion elbow step. Tell the author before rebuilding `~/bridge_ws_estop`.
2. **Ask the author decisions 2–4** in `docs/psi0_gates.md`: scope of gates 3, 5, 6, 7, 9; the brake profile; `robot_client.py`'s `L1`/`R1` mappings.
3. **Arm only with the remote connected** (author's request): first record `/lowstate.wireless_remote` with the remote off (read-only, robot latched or disarmed). Then design: the bridge refuses to arm without the remote; decide whether to damp when it is lost while armed. Get approval before C++.
4. **Console on the robot:** record the author's test results; then the roadmap "Robot console" checks. Start it in a foreground tmux pane (`--config` until `feat-robot-console` merges):
   ```bash
   tools/g1-console.sh --config ~/git/worktrees/humanoid-locoman-vla/feat-robot-console/benches/g1/console.yaml
   ```
   On 2026-10-20 it refused: two `G1_bridge` run (orphan pid 947798 from 10-19 17:22, parent `systemd --user`), and the bridge source (`gates.cpp`) is newer than the `~/bridge_ws_estop` binary.
5. **Gantry, with an operator:** G6-IMU (`imu_rpy_threshold: 0.05`, tilt -> damp).

## Blockers

None. Decisions 1–4 in `docs/psi0_gates.md` wait for the author.

## Gotchas (unpromoted)

- The robot's bridge runs from `~/bridge_ws_estop`: rebuild only `robot_console` there (`colcon build --packages-select robot_console`); bridge changes go to `~/bridge_ws_gates`.
- Run `tools/test.sh` with the robot cabled only in a namespace with no NIC:
  `unshare -rn bash -c 'ip link set lo up multicast on; ip route add 224.0.0.0/4 dev lo; set +u; source /opt/ros/humble/setup.bash; source <ws>/install/setup.bash; tools/test.sh --rmw cyclonedds'`.
- `ros2 topic` on the robot's topics needs a workspace sourced, or the message type is invalid.
- `tools/build.sh --workspace <ws>`: the clone is reached through a symlink, so workspace detection fails without it.
- Start `robot_console` in a foreground tmux pane: as a script's background job SIGINT is ignored, and processes stop only after the INT grace.
- Rename the worktree dir when no session runs in it: `git -C ~/github/humanoid_bridge worktree move ~/git/worktrees/humanoid_bridge/g1-estop-safety ~/git/worktrees/humanoid_bridge/g1-web-interface`, then re-point `~/bridge_ws_estop/src/humanoid_bridge` and `~/bridge_ws_gates/src/humanoid_bridge`.
