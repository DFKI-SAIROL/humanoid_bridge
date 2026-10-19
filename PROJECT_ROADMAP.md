# Project Roadmap — humanoid_bridge

> One per repo. Planned work only, one item per task. Finished items are
> deleted; git history is the log.

_Last updated: 2026-10-19 · adminsairol-MS-7E06_

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
  `dex3_probe --iface <if>` read mode is what answers it.
  **Done when:** `g1_stack/robot.py`'s UNCONFIRMED note on hand order is gone
  or corrected, and `hand_from_pose` is checked against a real hand.
- [ ] Decide whether the bridge should republish hand state as a
  `bridge_interface` message, so `g1_stack` can drop its raw-DDS hand reader.
  Blocked on whether the Jetson ever needs hand state.
  **Done when:** either the message exists and `g1_stack` uses it, or the
  duplication is documented as deliberate.

### Robot console

Built and fake-tested on `g1-web-interface` (`docs/robot_console.md`).

- [ ] Gantry checks: phone over the lab LAN with the token, arm at the measured pose, jog one joint, web E-STOP DAMP and HOLD, an observe-only run started from the page.
  **Done when:** each works on the G1 in the gantry, and the camera panel shows the Jetson preview.
- [ ] Lab process file: `humanoid-locoman-vla/benches/g1/console.yaml` (branch `feat-robot-console` there) runs psi0, camera, GEAR and the nodes.
  **Done when:** every entry starts and stops cleanly from the page on the lab PC.

### Upstream fixes — one branch each

- [ ] `fix/start-control-validate`: `startControlServiceCB_` answers `success: true` with an "invalid size" message unconditionally, then `initControl_` moves to `ready_q_` on a wrong length. **Done when:** a wrong length is rejected, the answer is false, and nothing moves.
- [ ] `fix/g1-default-pos-29`: G1 `_default_pos` is 27 long while kp/kd are 29 (`robot_client.py`). **Done when:** all three are 29 and a length check fails loudly.
- [ ] `fix/config-keymaps`: the stale keymap headers are corrected here (G1 and H1_2 advertised four keys the source comments out; T1 omitted `RT` from the start combo; H1 was right). Still to send upstream. **Done when:** the PR is open.
- [ ] `feat/g1-finish-control-release`: e-stop latch built on `g1-web-interface` (`docs/estop.md`), fake-tested only. **Done when:** gantry tests G0–G6 in `docs/estop.md` pass and the branch is merged.

### Cleanups

- [ ] Decide whether the service-call futures in `robot_client.py` should be awaited. Two are assigned and dropped (`init_control`, `stop_control`), currently ruff-suppressed as `F841`. **Done when:** the calls either await or explicitly document fire-and-forget, and the suppression is gone.
- [ ] Split `robot_client.py` (~600 lines): per-robot `default_pos`/kp/kd tables and `KeyMap` out of the client body. **Done when:** the tables are data in their own module and the ruff `E501` suppression for its commented-out keymap blocks is gone.
- [ ] `--interface sim` does not reach the bridge: MuJoCo listens on `rt/lowcmd`, which the bridge owns. **Done when:** a sim run drives the same client path as hardware, or the limitation is documented as permanent.
