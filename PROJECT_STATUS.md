# Project Status — humanoid_bridge

> One per repo. The *current* picture: goal, architecture, decisions, open
> questions. Finished work is removed; git history is the log.

_Last updated: 2026-10-24 · local (turing)_

## Goal

The lab's robot stack for Unitree (G1, H1, H1_2), Booster (T1) and AgiBot (X2) humanoids, in
two layers, so the bridge layer can go back to DFKI as PRs while the per-robot
deploy layers stay independent of it.

## Architecture

```
bridge/           upstream-bound: bridge_interface (msgs, srvs), robot_bridge (C++ + Python client)
robot_stacks/     per-robot deploy layers; g1/ holds g1_stack and g1_camera; sim/robot_sim: the MuJoCo sim of every robot
scripts/          environment setup, sourced not run
thirdparty/       vendored unitree_ros2, booster_ros2_interface, aimdk_msgs
```

Fork of `DFKI-SAIROL/humanoid_bridge`. `bridge/` imports nothing from
`robot_stacks/`; that one-way rule keeps either layer a `git subtree split`.

Deploy layers split by *owner*, not file type: code bound for a different
machine or a different Python floor is its own package.

## Key decisions

_One line each, dated, with a pointer (commit or PR)._

- 2026-09-09: Two layers, `bridge/` and `robot_stacks/<robot>/`, one-way dependency — commit `chore: restructure into bridge/ and robot_stacks/ layers`
- 2026-09-09: `robot_bridge_py` → `robot_bridge`; divergence from upstream paths accepted over deferring the rename — same commit
- 2026-09-09: Packaging metadata stays in `setup.py`; `pyproject.toml` is tool config only, because system setuptools is 59.6 and PEP 621 needs ≥61 — same commit
- 2026-09-09: Jetson-bound code stays its own package `g1_camera`, a sibling of `g1_stack` under `robot_stacks/g1/`: the package boundary makes the isolation structural rather than a convention a test has to defend, and it is shallower than folding it in — commit `refactor(g1_stack)`
- 2026-09-10: Dex3 hands get their own bridge path — two per-side topics, own enable, own 100 Hz timer — rather than joining the 29-joint body vector, because the hands are a separate device with their own state topic and rate; `docs/hands_path.md`
- 2026-09-10: a hand release is limp (`kp = kd = 0`), deliberately unlike the body's hold-last-pose, because a hand here holds nothing critical and limp cannot cook a stalled finger — same doc
- 2026-09-10: `adapters/gear_hands.py` is a pass-through, not a remap: GEAR's `HandCommandSender` already takes DDS order, and the goal-order remap belongs in `g1_stack.gear.goal.hand_from_pose` where the goal order is defined — same doc
- 2026-09-10: `BridgeCore::checkMotorCmd_` uses `std::abs`, not the original's unqualified `abs`, which resolved to the integer overload and truncated any |value| < 1 to 0. The body path's "unreasonably large" guard was much weaker than it read; this is a behaviour change on the body path — commit `27f93c9`
- 2026-09-10: build and test are scripts, not doc paragraphs: `tools/` holds executables, `scripts/` stays sourced-only. Build deps that rosdep cannot pull now live here rather than only in the consumer repo's Dockerfile — `docs/handover-audit.md`
- 2026-09-10: `fake_dex3.py`'s `--stale-after`/`--hot-after` clocks start at the first `/dex3/left/cmd`, not process start, so the documented 3.0-4.0 window means what the prose always claimed — same doc
- 2026-10-19: every body stop goes through one latch: operator e-stops `damp` (`kp=0, kd=8`) and `hold` (measured pose), both latched until `/estop_reset`; `/stop_control` holds without a latch; faults latch. Replaces `/lowcmd` silence after a stop — `docs/estop.md`
- 2026-10-20: the bridge is UI-free; the G1 web page moved to `SiweiJu/g1_web_commander` (it was `bridge/robot_console`, robot-generic and upstream-bound: no longer); contract: the ROS graph, `bridge_interface` types, `robot_bridge.cmd_client` — removal commit on `g1-web-interface`
- 2026-10-19: G1 remote keys come from `/lowstate.wireless_remote`, not `/wirelesscontroller`, which sends nothing in debug mode; e-stop combos `R1+R2` damp, `L1+R1` hold (swapped 2026-10-23, PR #32) — `docs/estop.md`
- 2026-10-20: the bridge holds one robot envelope for every model (range clamp, speed limit, gains, speed trip); model-specific shaping lives in each model's adapter node — `docs/psi0_gates.md`
- 2026-10-20: G1 arming needs the remote's header; a latch goes silent 1.2 s after `/lowstate` stops, so a restarted robot gets no old frames — `docs/estop.md`
- 2026-10-22: upstream syncs land as merge commits, never rebase or squash, so upstream commit IDs stay and later syncs and upstream PRs stay simple — PR "Merge DFKI-SAIROL/humanoid_bridge main"
- 2026-10-22: `BridgeCore::checkState_` calls upstream's virtual `checkStateFreshness_` for state loss; the X2 overrides it, the G1 path is unchanged — same PR
- 2026-10-22: another `/lowcmd` sender while ARMED latches damp (`other_lowcmd`) and the bridge keeps sending damp frames, not silence: silence would hand the robot to an unknown sender and disable the remote e-stop combos; revisit after the gantry measurement (roadmap) — PR "fix(bridge): latch damp when another program sends /lowcmd (B11b)"
- 2026-10-22: start_control, ready and zero services exist only after the bridge's start-up waits; e-stop, stop and reset exist at once — PR #10 (X3)
- 2026-10-22: Ready and Zero count as arming: they run the remote and `/lowcmd` checks — PR #13 (B2)
- 2026-10-22: safety timers run on the steady clock; message headers keep wall time — PR #16 (B5)
- 2026-10-22: G1 `estop.require_remote` defaults to true; turn it off only in a sim-only params file — PR #26 (B11a)
- 2026-10-22: parameters are read-only at run time; `start_control` refuses `default_kp`/`default_kd` — PRs #23, #22 (B11d, B11c)
- 2026-10-22: a clean shutdown while ARMED latches damp (`shutdown`) and sends it for 1 s — PR #25 (B9)
- 2026-10-22: the gate is `tools/test.sh --docker`; a stage that skips exits 2 unless `--allow-skip` — PR "chore: bridge codebase health" (BT12, BT1)
- 2026-10-22: `camera_server` binds `--bind`, default the Jetson's 192.168.123.164, never 0.0.0.0 — same PR (B12b)
- 2026-10-24: G1 `/robot_link` tells a link drop (`BACK … no_restart`, tick moved on) from a reboot (`RESTARTED`); after `RESTARTED` arming and `/estop_reset` stay refused until a new bridge (bring-up releases) — `docs/estop.md` § Robot link, PR #36
- 2026-10-25: `g1_sim` became `robot_sim` (`--robot g1`; T1, Go2 next): one physics, one module per robot at its vendor topics; inputs `/robot_sim/*`, buttons by name — `robot_stacks/sim/robot_sim/README.md`, multi-robot track (g1_web_commander `docs/multi-robot.md`)
- 2026-10-24: `g1_sim` motors go passive 0.4 s after the last `/lowcmd`, as the G1 (G0); its reboot starts a fake motion service that unitree_sdk2py releases — `robot_stacks/sim/robot_sim/README.md`, PR #36
- 2026-10-24: Ready (`/ready_position_control`) also ramps each Dex3 hand with fresh state to `hand_ready_q` (open) and holds it; a hand failure never blocks the body; arming stays body-only — `docs/hands_path.md`, PR from `feat-hands-ready-pos`
- 2026-10-22: the G1 sim is an own MuJoCo node `g1_sim` on mjlab `g1.xml`, not GEAR's `base_sim` (needs rclpy the SIMPLE image lacks, no remote header, `mode_machine` 0, other model) — `robot_stacks/sim/robot_sim/README.md`
- 2026-09-09: `examples/` sits inside `robot_bridge`, the package it demonstrates, so a subtree split of `bridge/` carries it; excluded from lint as vendored upstream code — same commit

## Roadmap

Planned work lives in `PROJECT_ROADMAP.md`; this file keeps only the pointer.

## Open questions

- Does `robot_stacks/g1/` stay in this repo or become its own? DFKI's call, deferred.
- Dex3 left/right read order: which half of the 14-vector is which hand, and which motor index is which finger. Dex3 is mounted on the lab G1 (2026-10-20) and sends state; not yet checked. `dex3_probe` read mode answers it.
- Remote lost while the G1 is armed: damp, hold, or only a warning? — `docs/estop.md`
- Self-collision: no gate; the joint ranges let an arm reach the head — `docs/psi0_gates.md`
