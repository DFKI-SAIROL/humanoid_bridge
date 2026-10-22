# Handoff — humanoid_bridge / sync-upstream

> The baton. Present tense only: where the work stands *now* and what the
> next session should do. Overwrite it each session — never append. History
> lives in git and rationale in commit bodies, not here. Before the task
> ends, promote anything durable (project status, repo instructions, commit
> body) and delete this file from the branch: a finished task hands
> nothing off, and merged, a leftover baton strays onto the default branch.

- **Repo:** humanoid_bridge
- **Branch:** `sync-upstream`
- **Worktree:** /home/lukas-mueller/git/worktrees/humanoid_bridge/sync-upstream
- **Last updated:** 2026-10-22 11:30 CEST · local (turing)

## State

Not started. **Goal (Lukas, 2026-10-22):** merge `DFKI-SAIROL/humanoid_bridge` `main` into this fork's `main` before the codebase-health fixes start. This session opens the PR; Lukas merges.

- **Upstream:** https://github.com/DFKI-SAIROL/humanoid_bridge (not linked as a GitHub fork: add it as remote `upstream`). Merge base `b543634` ("add H1_2 bridge"); fork 61 ahead (head `9fc7cce`, after PR #7), upstream 32 ahead.
- **The 32 commits add the AgiBot X2:** X2 bridge, node, config, Python client branch; `thirdparty/aimdk_msgs`; examples (incl. a MuJoCo viewer); `setup_x2.sh`; CMake option `BUILD_AGIBOT_X2` (default OFF).
- **Decided: a merge commit**, not rebase or squash, so upstream's commit IDs stay and later syncs and upstream PRs stay simple. The PR body asks Lukas to use "Create a merge commit".
- **Trial merge (`git merge-tree`, 2026-10-22) conflicts:**
  1. `robot_bridge/{include/X2_bridge.hpp, params/X2_config.yaml, src/X2_bridge.cpp, src/x2_node.cpp}` land in the old path: the fork moved the dir to `bridge/robot_bridge/`. Move them; check `example/X2/…` and the Python client follow the fork's layout too.
  2. `bridge_core.hpp` / `bridge_core.cpp`: upstream splits `checkState_()` into a virtual `bool checkStateFreshness_()` (11 lines). The fork's `checkState_()` returns `estop::Cause` (e-stop latch). Keep the fork's version, call the virtual freshness check inside it, map stale to the cause the fork already uses. **The G1's behaviour must not change.**
  3. `robot_client.py`: moved and changed in the fork; port upstream's X2 branch (+276 lines) by hand.
  4. `CMakeLists.txt`, `package.xml` auto-merge: check `aimdk_msgs` is not a hard dependency of G1 builds.
  5. `X2_bridge.cpp` targets upstream's core (no e-stop latch, no gates): with `-DBUILD_AGIBOT_X2=ON` it may not compile. Adapt minimally. The X2 cannot be run-tested (no robot, no fake): say so in the PR.
- **Also in this PR:** PR #7's `HANDOFF.md` landed on `main` by mistake (`git show origin/main:HANDOFF.md`). Move its open point, the gantry acceptance test, to a `PROJECT_ROADMAP.md` item. Deleting this branch's `HANDOFF.md` at the end then removes the stray file from `main` on merge.

## Next action

1. `git remote add upstream https://github.com/DFKI-SAIROL/humanoid_bridge.git && git fetch upstream && git merge --no-ff upstream/main`; resolve as listed in State.
2. Build in docker with the commander's test image `g1-web-commander-test:humble` (see `run_in_docker` in g1_web_commander `tools/test.sh`): once with `BUILD_AGIBOT_X2` OFF, once ON.
3. Gates:
   - this repo's `tools/test.sh` stages in that image: cpp, wire, estop, gates, hands must PASS (unit skips there for `zmq`/`msgpack`: state it);
   - g1_web_commander `tools/test.sh --docker` with `BRIDGE_CLONE` = this worktree.
4. Add the roadmap item, delete this `HANDOFF.md`, push, open the PR to `main`: "Merge DFKI-SAIROL/humanoid_bridge main: AgiBot X2 (32 commits)". Body: what the commits add; each conflict and its resolution; "X2 not run-tested"; gate results; "merge with a merge commit".

**Done when:** the PR is open, the gates pass, and this handoff is deleted from the branch.

## Blockers

None.

## Gotchas (unpromoted)

- Never push to `main`: this repo has no pre-push hook.
- Agent sandbox: `docker`, `git fetch`/`push` and git writes to this clone need the unsandboxed retry.
- Bridge gate in the commander image (2026-10-21): `docker run --rm --init --network none --user $(id -u):$(id -g) -e HOME=/tmp -v ~/.cache/g1_web_commander/test_ws:/ws -v "$PWD":/ws/src/humanoid_bridge:ro -w /ws/src/humanoid_bridge g1-web-commander-test:humble bash -c '. /opt/ros/humble/setup.bash; . /ws/install/setup.bash; tools/test.sh'`. Rebuild that workspace from this worktree first, or the cpp stages test an old build.
- `mujoco-sim` and `jetson-link-watcher` sit on pre-rebase commits (`4b52cca`): not this task's branches.
- After this merges, the codebase-health session (g1_web_commander `chore-codebase-health`) starts the bridge `fix/` branches from `main`.
