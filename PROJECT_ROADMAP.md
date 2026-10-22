# Project Roadmap — humanoid_bridge

> One per repo. Planned work only, one item per task. Finished items are
> deleted; git history is the log.

_Last updated: 2026-10-22 · turing_

## Items

### Handover

- [ ] Delete `docs/handover-audit.md`: its plan (P0–P2) is done.
  **Done when:** the file is gone and `PROJECT_STATUS.md` no longer points at it.

### Hand path

Built, compiled and verified against the fakes (`tools/test.sh`, all stages
pass on both CycloneDDS and Fast DDS). Design in `docs/hands_path.md`. What is
left needs hardware:

- [ ] Confirm the Dex3 read order against hardware once the hands are mounted:
  bend one left-hand finger and watch which side and which motor index answers.
  `dex3_probe --iface <if>` read mode is what answers it. Also bend the right
  index finger: Unitree's enum (psi0 teleop) puts right index before middle on
  motors 3–6, our config and `DEX3_ORDER` put middle first.
  **Done when:** `g1_stack/robot.py`'s UNCONFIRMED note on hand order is gone
  or corrected, and `hand_from_pose` is checked against a real hand.
- [ ] Decide whether the bridge should republish hand state as a
  `bridge_interface` message, so `g1_stack` can drop its raw-DDS hand reader.
  Blocked on whether the Jetson ever needs hand state.
  **Done when:** either the message exists and `g1_stack` uses it, or the
  duplication is documented as deliberate.

### psi0 gates

Decided and fake-tested on `g1-web-interface` (`docs/psi0_gates.md`).

- [ ] Leg speed limit: measure GEAR's leg target speeds in the MuJoCo rehearsal; `q_rate_limit_default` is 6.0 until then.
  **Done when:** the default is set from the measured max plus margin.
- [ ] IMU trip: measure the standing tilt under GEAR and psi0 in the rehearsal, then lower `imu_rpy_threshold` from 1.0 to about 0.5.
  **Done when:** the new value is in `G1_config.yaml` and G6-IMU passes on the gantry with it.

### GEAR on the robot

- [ ] GEAR's robot mode sends waist pitch (motor 14) kd 5 − 10 = −5 (`decoupled_wbc` `configs.py:64-66`, in the pinned SIMPLE image); `adapters/gear_wbc.py` forwards it and the bridge clips it to 0 (`bridge_core.cpp:697-703`). **Done when:** the intended robot kd is known and `/robot_cmd` carries it for motor 14, checked with `ros2 topic echo /robot_cmd`.
- [ ] GEAR waits forever for Dex3 state on a G1 without hands: `with_hands` defaults to True (`configs.py:108`); neither `g1_stack/nodes/gear_controller.py` nor the commander workspace passes `--no-with-hands`, and robot-mode `calibrate_hands` blocks with no timeout. **Done when:** a hand-less G1 run reaches "GEAR -> /robot_cmd at 50 Hz".

- [ ] Dex3 finger gains: GEAR's hand sender (`decoupled_wbc` `command_sender.py:130-133`, forwarded by `adapters/gear_hands.py`) sends kp 2/1, kd 0.5/0.2; SIMPLE trains and evaluates with kp 5/2.5, kd 1 (`G1Sonic.apply_action`). A bridge try of psi0 `g1-xmovepick` (g1-pick-smoke ep 0) failed the grasp with GEAR's gains and succeeded in-process. **Done when:** the gains the real Dex3 needs are measured on the robot and the sim or the sender matches them.
- [ ] Robot rehearsal on the real-robot branch, after both items above: the full G1 path (workspace `psi0` with a real-robot checkpoint → `psi0_gear_loop` → GEAR robot mode → G1_bridge) against `g1_sim` instead of the G1. Needs `g1_sim` to publish `rt/secondary_imu` and answer the motion switcher ("no mode"), the workspace's camera host and DDS interface from the commander's mode, and the multicast patch on `lo` for the loop node. **Done when:** a rehearsal from the page reaches GEAR's loop and moves `g1_sim` through the loop node.

### Gantry acceptance

- [ ] Run the post-PR #7 acceptance test on the gantry: `docs/estop.md`, "Gantry acceptance, after PR #7". Then switch the robot's bridge to that build.
  **Done when:** rows 1–8 pass, the results replace the table in `docs/estop.md`, and the robot runs the `main` build.

### Link loss

- [ ] Jetson link watcher: a process on the Jetson freezes the robot (`hold`, `docs/estop.md`) when the link to the bridge on the lab PC drops, Ethernet or WiFi. It must act within 0.4 s: the motors go passive 0.4 s after `/lowcmd` stops (G0), so a link drop today drops a standing robot.
  **Done when:** on the gantry, armed, a pulled Ethernet cable and a dropped WiFi link each leave the robot holding its pose until a reset, and the watcher sends nothing while the bridge runs or after a normal disarm.

- [ ] Measure on the gantry what Unitree's motion service does when it takes over `/lowcmd` while the bridge is latched `other_lowcmd` (damp frames keep going, `docs/estop.md`).
  **Done when:** the result is in `docs/estop.md`, and the `other_lowcmd` rule (keep damp vs go silent) is confirmed or changed.

### Upstream fixes — one branch each

- [ ] `fix/g1-default-pos-29`: G1 `_default_pos` is 27 long while kp/kd are 29 (`robot_client.py`). **Done when:** all three are 29 and a length check fails loudly.
- [ ] `fix/config-keymaps`: the keymap headers in `params/*.yaml` are corrected here: each key names its handler, `robot_client.py` or the bridge. Still to send upstream. **Done when:** the PR is open.
- [ ] `feat/g1-finish-control-release`: e-stop latch built on `g1-web-interface` (`docs/estop.md`), fake-tested only. **Done when:** gantry tests G0–G6 in `docs/estop.md` pass and the branch is merged.

### Cleanups

- [ ] Decide whether the service-call futures in `robot_client.py` should be awaited. Two are assigned and dropped (`init_control`, `stop_control`), currently ruff-suppressed as `F841`. **Done when:** the calls either await or explicitly document fire-and-forget, and the suppression is gone.
- [ ] Split `robot_client.py` (~600 lines): per-robot `default_pos`/kp/kd tables and `KeyMap` out of the client body. **Done when:** the tables are data in their own module and the ruff `E501` suppression for its commented-out keymap blocks is gone.
- [ ] `--interface sim` does not reach the bridge: MuJoCo listens on `rt/lowcmd`, which the bridge owns. **Done when:** a sim run drives the same client path as hardware, or the limitation is documented as permanent.
