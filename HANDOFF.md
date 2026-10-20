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
- **Last updated:** 2026-10-20 · local (adminsairol-MS-7E06)

## State

- **psi0 gates (task 2):** decided with the author and built (`docs/psi0_gates.md`): gates 1, 2, 4, 5, 6, 9, client default order 1, `L1`/`R1` client mappings gone. Unit and fake tests pass. Built in `~/bridge_ws_gates` only.
- **Remote:** arming needs the remote's header (`estop.require_remote`); console pill `remote` / `no remote`. Remote lost while armed: open question (`PROJECT_STATUS.md`).
- **Latch after state loss:** silent 1.2 s after `/lowstate` stops (`docs/estop.md`).
- **Review fixes:** arming refused while another `/lowcmd` publisher runs; `start_control` values over 0.1 rad outside the range refused; a `/robot_cmd` with `interpolation_order` outside [0, 1] is dropped (it ended the bridge before); fake test scripts pin DDS to loopback; the release retries 5 times.
- **Bridge exit hang fixed:** with the NIC down, Cyclone's `dds_delete` never returned; `g1_node.cpp` ends the process 3 s after "G1 Bridge stopped.".
- **`tools/g1-bringup.sh`:** sources the environment itself, releases with the lab venv, refuses two bridges, `--stop` (SIGKILL fallback).
- **`tools/g1-console.sh`:** new check "G1 limits vs URDF" (`tools/check-g1-limits.py`).
- **Robot:** in the gantry. The bridge on the robot still runs the old binary from `~/bridge_ws_estop`, or nothing.

## Next action: the author's acceptance test (robot in the gantry)

Setup (all from the gates workspace, not `~/bridge_ws_estop`):
```bash
B=~/bridge_ws_gates/src/humanoid_bridge/tools
$B/g1-bringup.sh --stop
$B/g1-bringup.sh --workspace ~/bridge_ws_gates --iface enp4s0
# tmux pane robot-console:
$B/g1-console.sh --workspace ~/bridge_ws_gates \
  --config ~/git/worktrees/humanoid-locoman-vla/feat-robot-console/benches/g1/console.yaml
# for ros2 / g1-step-test.py in another pane:
set +u; source /opt/ros/humble/setup.bash; BRIDGE_IFACE=enp4s0 source ~/bridge_ws_gates/src/humanoid_bridge/scripts/setup_unitree.sh
```

| # | Do | Expect |
|---|---|---|
| 1 | read the console start output | `G1 limits vs URDF` ok |
| 2 | remote off, click Arm | refused "remote not connected"; pill `no remote` |
| 2b | after a robot restart, before the release: Arm | refused "other /lowcmd publisher" |
| 3 | remote on, Arm at measured pose | pill `remote`; armed, nothing moves |
| 4 | `ros2 service call /start_control bridge_interface/srv/SetDefaultPosition "{default_position: [0.0, 0.0]}"` | `success: false`, nothing moves |
| 5 | armed, no jog: `$B/g1-step-test.py --joint R_ELBOW --delta 0.5` | elbow ramps, no snap; both lines PASS (<= 3 rad/s) |
| 6 | console DAMP, HOLD; remote `L1+R1`, `R1+R2`; RESET | as on 10-19 |
| 7 | armed, pull the Ethernet cable 5 s, plug it in | bridge log `No /lowstate for 1.2 s: … silent`; robot stays passive after the cable is back; RESET -> `DISARMED` |
| 8 | `$B/g1-bringup.sh --stop` | `stopped`, no SIGKILL line |

After it passes: switch the robot's bridge to the new build (rebuild `~/bridge_ws_estop`, tell the author first, or keep `--workspace ~/bridge_ws_gates`), then G6-IMU (`imu_rpy_threshold: 0.05`, tilt -> damp).

## Also open

- The feat-g1-rollout session (humanoid-locoman-vla) is told: psi0's 0.12 rad leash goes into `psi0_bridge_loop`. Check that it lands there.
- humanoid-locoman-vla `feat-robot-console`: `benches/g1/console.yaml` has an uncommitted change (urdf -> `g1_mjlab/g1.urdf`) from the other session.

## Blockers

None.

## Gotchas (unpromoted)

- The robot's bridge runs from `~/bridge_ws_estop`: rebuild only `robot_console` there; bridge changes go to `~/bridge_ws_gates`.
- Run `tools/test.sh` with the robot cabled only in a namespace with no NIC:
  `unshare -rn bash -c 'ip link set lo up multicast on; ip route add 224.0.0.0/4 dev lo; set +u; source /opt/ros/humble/setup.bash; source <ws>/install/setup.bash; tools/test.sh --rmw cyclonedds'`.
- `ros2 topic` on the robot's topics: source `setup_unitree.sh` too, and use `--no-daemon` after a link drop (a stale daemon reports 0 publishers).
- `BRIDGE_IFACE=x source …` does not export it: pass `--iface` to scripts.
- After a robot boot the camera can send empty frames (`/health` ok, frame 503): restart `video_hub_pc4` with the SDK's `RobotStateClient.ServiceSwitch` (off, 3 s, on).
- Start `robot_console` in a foreground tmux pane: as a script's background job SIGINT is ignored.
- Rename the worktree dir when no session runs in it: `git -C ~/github/humanoid_bridge worktree move ~/git/worktrees/humanoid_bridge/g1-estop-safety ~/git/worktrees/humanoid_bridge/g1-web-interface`, then re-point `~/bridge_ws_estop/src/humanoid_bridge` and `~/bridge_ws_gates/src/humanoid_bridge`.
