# Project Roadmap — humanoid_bridge

> One per repo. Planned work only, one item per task. Finished items are
> deleted; git history is the log.

_Last updated: 2026-10-20 · lukas-mbp_

## Items

### Handover

- [ ] Finish the handover pass in `docs/handover-audit.md`: bring-up scripts,
  the `bridge/` conftest, moving bridge-behaviour facts out of the consumer
  repo, and the dead-code cleanup.
  **Done when:** every P1 and P2 item in that plan is done and the audit is
  deleted.

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

### Link loss

- [ ] Jetson link watcher: a process on the Jetson freezes the robot (`hold`, `docs/estop.md`) when the link to the bridge on the lab PC drops, Ethernet or WiFi. It must act within 0.4 s: the motors go passive 0.4 s after `/lowcmd` stops (G0), so a link drop today drops a standing robot.
  **Done when:** on the gantry, armed, a pulled Ethernet cable and a dropped WiFi link each leave the robot holding its pose until a reset, and the watcher sends nothing while the bridge runs or after a normal disarm.

### Upstream fixes — one branch each

- [ ] `fix/g1-default-pos-29`: G1 `_default_pos` is 27 long while kp/kd are 29 (`robot_client.py`). **Done when:** all three are 29 and a length check fails loudly.
- [ ] `fix/config-keymaps`: the stale keymap headers are corrected here (G1 and H1_2 advertised four keys the source comments out; T1 omitted `RT` from the start combo; H1 was right). Still to send upstream. **Done when:** the PR is open.
- [ ] `feat/g1-finish-control-release`: e-stop latch built on `g1-web-interface` (`docs/estop.md`), fake-tested only. **Done when:** gantry tests G0–G6 in `docs/estop.md` pass and the branch is merged.

### Cleanups

- [ ] Decide whether the service-call futures in `robot_client.py` should be awaited. Two are assigned and dropped (`init_control`, `stop_control`), currently ruff-suppressed as `F841`. **Done when:** the calls either await or explicitly document fire-and-forget, and the suppression is gone.
- [ ] Split `robot_client.py` (~600 lines): per-robot `default_pos`/kp/kd tables and `KeyMap` out of the client body. **Done when:** the tables are data in their own module and the ruff `E501` suppression for its commented-out keymap blocks is gone.
- [ ] `--interface sim` does not reach the bridge: MuJoCo listens on `rt/lowcmd`, which the bridge owns. **Done when:** a sim run drives the same client path as hardware, or the limitation is documented as permanent.
