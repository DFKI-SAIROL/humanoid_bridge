# Handoff — humanoid_bridge / feat-robot-link

> The baton. Present tense only: where the work stands *now* and what the
> next session should do. Overwrite it each session — never append. History
> lives in git and rationale in commit bodies, not here. Before the task
> ends, promote anything durable (project status, repo instructions, commit
> body) and delete this file from the branch: a finished task hands
> nothing off, and merged, a leftover baton strays onto the default branch.

- **Repo:** humanoid_bridge (+ `g1_web_commander`, same branch name)
- **Branch:** `feat-robot-link`
- **Worktree:** /home/lukas-mueller/git/worktrees/humanoid_bridge/feat-robot-link
- **Last updated:** 2026-10-24 · local (turing)

## State

- Ready opens the Dex3 hands: merged in #35 (965fc7b).
- Feature 2, bridge repo half: done, PR #36 open. Spec: `docs/estop.md` § Robot link; sim side: `robot_stacks/g1/g1_sim/README.md`.
- Gate: `tools/test.sh --docker` passes; `tools/test.sh --sim` (sim, sim-link) passes in the g1-sim image (rebuild it: it now has `unitree_sdk2py`).
- Not started: Feature 1, the commander half of Feature 2.
- Commander branch `feat-robot-link` does not exist yet: `cd ~/github/g1_web_commander && vibe start feat-robot-link`.
- Merge order: the bridge PR first; the commander's sim config needs `g1-bringup.sh --sim`.

## Feature 1 — Quit from the page (commander only)

- Button on the controller tab; confirm lists what stops; then the normal `shutdown()` (releases the lock); page shows "commander stopped".
- Allowed only while the bridge sends no `/lowcmd`: not running, `DISARMED`, `LATCHED … silent`.
- Refused while: `ARMED`, `STOPPED hold`, `LATCHED` (not silent), joint probe active, any `commands_robot` process runs. 409 with the reason.
- Docker sim: the container exits with the commander (`--rm`).
- Test: refused while ARMED/held/probe/policy; accepted DISARMED; process exits, lock free.

## Feature 2 — robot link: reboot and link loss

Commander:
- `RESTARTED` (+ `other_lowcmd` as confirmation): banner + "Restart bring-up": always `G1_bridge` stop/start (normal confirms; start releases); camera only if `/health` fails or its ssh session ended. Refused while a `commands_robot` process runs, naming it.
- `BACK … no_restart`: "link back after X s, no reboot: Reset is safe" + Reset.
- Warning while a `commands_robot` process runs and `/robot_link` is not `OK`.
- Arm gate: Arm and Ready refused while a `commands_robot` process runs that already ran during the gap ("restart <proc> first: its state is from before the gap").
- Sim config bridge entry → `g1-bringup.sh --foreground --sim`; sim panel buttons "Drop link 3 s", "Reboot robot".
- Delete the commander roadmap item "Name a robot restart" when this lands.

## Next action

Commander: Feature 1 (Quit) on `g1_web_commander` `feat-robot-link` (author starts it); its Feature 2 half after #36 merges. Delete this file before #36 merges.

## Blockers

None. Real-G1 tick reset at boot is unverified (roadmap).

## Gotchas (unpromoted)

- The sandbox shows `/dev/null` masks (`.bashrc`, `.claude/…`) as untracked files in the worktree: never `git add -A`.
