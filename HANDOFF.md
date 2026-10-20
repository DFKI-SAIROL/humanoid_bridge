# Handoff — humanoid_bridge / g1-web-interface

> The baton. Present tense only: where the work stands *now* and what the
> next session should do. Overwrite it each session — never append. History
> lives in git and rationale in commit bodies, not here. Before the task
> ends, promote anything durable (project status, repo instructions, commit
> body) and delete this file from the branch: a finished task hands
> nothing off, and merged, a leftover baton strays onto the default branch.

- **Repo:** humanoid_bridge
- **Branch:** `g1-web-interface` (lab PC worktree dir still `g1-estop-safety`; see Gotchas)
- **Worktree:** lab PC /home/lukasm/git/worktrees/humanoid_bridge/g1-estop-safety; Mac ~/github/humanoid_bridge
- **Last updated:** 2026-10-20 · local (lukas-mbp)

## State

- **UI split:** the web console left this repo (`f58eedf`). It is `SiweiJu/g1_web_commander` now: PR SiweiJu/g1_web_commander#1 (branch `chore-init`). This branch has a draft PR to `main`; it waits for the acceptance test.
- **psi0 gates, remote rule, silent latch, review fixes, exit hang:** built, unit and fake tests pass (`docs/psi0_gates.md`, `docs/estop.md`). Built in `~/bridge_ws_gates` only.
- **Robot:** in the gantry. The bridge on the robot still runs the old binary from `~/bridge_ws_estop`, or nothing.

## Next action: the author's acceptance test (robot in the gantry)

Setup, once, on the lab PC (the commander builds in the same workspace):
```bash
git clone git@github.com:SiweiJu/g1_web_commander.git ~/github/g1_web_commander
git -C ~/github/g1_web_commander config core.hooksPath .githooks
git -C ~/github/g1_web_commander worktree add ~/git/worktrees/g1_web_commander/chore-init chore-init
ln -s ~/git/worktrees/g1_web_commander/chore-init ~/bridge_ws_gates/src/g1_web_commander
(cd ~/bridge_ws_gates && colcon build --packages-select g1_web_commander)
mv ~/.local/share/robot_console ~/.local/share/g1_web_commander    # or run its fetch_assets.sh
~/bridge_ws_gates/src/g1_web_commander/tools/test.sh --workspace ~/bridge_ws_gates   # in the no-NIC namespace (Gotchas)
```

Each run:
```bash
B=~/bridge_ws_gates/src/humanoid_bridge/tools
C=~/bridge_ws_gates/src/g1_web_commander/tools
$B/g1-bringup.sh --stop
$B/g1-bringup.sh --workspace ~/bridge_ws_gates --iface enp4s0
# tmux pane commander (new token: open the printed URL again):
$C/g1-commander.sh --workspace ~/bridge_ws_gates --adapter humanoid_locoman_vla
# for ros2 / g1-step-test.py in another pane:
set +u; source /opt/ros/humble/setup.bash; BRIDGE_IFACE=enp4s0 source ~/bridge_ws_gates/src/humanoid_bridge/scripts/setup_unitree.sh
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
| 6 | page DAMP, HOLD; remote `L1+R1`, `R1+R2`; RESET | as on 10-19 |
| 7 | armed, pull the Ethernet cable 5 s, plug it in | bridge log `No /lowstate for 1.2 s: … silent`; robot stays passive after the cable is back; RESET -> `DISARMED` |
| 8 | `$B/g1-bringup.sh --stop` | `stopped`, no SIGKILL line |

After it passes: set the draft PR to ready and merge it; switch the robot's bridge to the new build (rebuild `~/bridge_ws_estop`, tell the author first, or keep `--workspace ~/bridge_ws_gates`); then G6-IMU (`imu_rpy_threshold: 0.05`, tilt -> damp).

## Also open

- The feat-g1-rollout session (humanoid-locoman-vla) is told: psi0's 0.12 rad leash goes into `psi0_bridge_loop`. Check that it lands there.
- humanoid-locoman-vla `feat-robot-console`: delete `benches/g1/console.yaml` (it moved to the commander's `adapters/humanoid_locoman_vla/`) and point its roadmap item there. On the lab PC, discard the file's uncommitted URDF change first: `git checkout -- benches/g1/console.yaml`.

## Blockers

None.

## Gotchas (unpromoted)

- The robot's bridge runs from `~/bridge_ws_estop`; bridge changes go to `~/bridge_ws_gates`.
- Run `tools/test.sh` with the robot cabled only in a namespace with no NIC:
  `unshare -rn bash -c 'ip link set lo up multicast on; ip route add 224.0.0.0/4 dev lo; set +u; source /opt/ros/humble/setup.bash; source <ws>/install/setup.bash; tools/test.sh --rmw cyclonedds'`.
- `ros2 topic` on the robot's topics: source `setup_unitree.sh` too, and use `--no-daemon` after a link drop (a stale daemon reports 0 publishers).
- `BRIDGE_IFACE=x source …` does not export it: pass `--iface` to scripts.
- After a robot boot the camera can send empty frames (`/health` ok, frame 503): restart `video_hub_pc4` with the SDK's `RobotStateClient.ServiceSwitch` (off, 3 s, on).
- Rename the worktree dir when no session runs in it: `git -C ~/github/humanoid_bridge worktree move ~/git/worktrees/humanoid_bridge/g1-estop-safety ~/git/worktrees/humanoid_bridge/g1-web-interface`, then re-point `~/bridge_ws_estop/src/humanoid_bridge` and `~/bridge_ws_gates/src/humanoid_bridge`.
