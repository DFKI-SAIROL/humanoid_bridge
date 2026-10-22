# Handoff — humanoid_bridge / feat-sim-closed-loop

> The baton. Present tense only: where the work stands *now* and what the
> next session should do. Overwrite it each session — never append. History
> lives in git and rationale in commit bodies, not here. Before the task
> ends, promote anything durable (project status, repo instructions, commit
> body) and delete this file from the branch: a finished task hands
> nothing off, and merged, a leftover baton strays onto the default branch.

- **Repo:** humanoid_bridge (baton for all three repos)
- **Branch:** `feat-sim-closed-loop` in humanoid_bridge, humanoid-locoman-vla, g1_web_commander (worktree dir here still `mujoco-sim`: `git worktree move` it when no session runs in it)
- **Worktrees:** `~/git/worktrees/humanoid_bridge/mujoco-sim`, `~/git/worktrees/humanoid-locoman-vla/feat-sim-closed-loop`, `~/git/worktrees/g1_web_commander/feat-sim-closed-loop`; all rebased on main (bridge 1fec7c7, commander 16d1786; lab main unmoved) and pushed
- **Last updated:** 2026-10-22 · local (turing)

## State

- **Goal:** one `lab try` of a checkpoint with G1_bridge in the loop (`--controller-path bridge`), from terminal or commander page (`try_bridge`); never counted. Done-when met; docs: humanoid-locoman-vla `docs/bridge-try.md`.
- **PRs open** (Lukas merges, in this order): bridge, commander, humanoid-locoman-vla. Each still carries `HANDOFF.md`: `git rm` it on all three branches before merge.
- **Gates after the rebase:** bridge `--docker` all PASS, bridge `--sim` PASS; commander `--docker` unit 346 + console PASS, commander `--sim` PASS; lab 667 OK.
- **Bridge try after the rebase, episode 0 of g1-pick-smoke, headless:** terminal success at 246; page timeout at 300. Same flags; the bridge path runs on the wall clock, so outcomes vary run to run.
  - Still open: the bridge robot walks ~4 cm further (GEAR's gait through the bridge) and starts ~1.8 cm closer; it touches the box.
  - `--live` on a MuJoCo suite halves object weight (viewer clears SIMPLE's `xfrc_applied` pseudo gravity): live and headless outcomes differ, also for plain `try`.
- **Isaac live view:** plain `try` on a `mujoco_isaac` suite with `--live` opens Isaac Sim's window (verified: g1-pick-isaac-headless, success 174). The bridge path stays MuJoCo-only (Isaac ~167 ms/step vs 20 ms real time).
- **`hooks/isaac-view.py` (WIP, opt-in `--hook isaac-view`):** head-camera viewport + hidden editor UI work; the timeline-resume works (`resumed` note). Bug: with the hook, every camera frame (policy input, stored video) freezes at the first frame, without any pause (runs `r709240`, `rbf59e7` vs hook-free `r441226`).
- **Space key in Isaac's window:** pauses Isaac's timeline; camera frames freeze, MuJoCo runs on.
- **Page dev instance:** `/tmp/claude-1000/-home-lukas-mueller-git-worktrees-humanoid-bridge-mujoco-sim/4769ec76-c126-47e2-be34-66947e699c1f/scratchpad/start-page.sh` (container `g1-cmdr-e2e`, port 18090, lost on reboot or a docker upgrade); build `~/.cache/g1_sim/hostws` first (`tools/build.sh ... robot_bridge g1_sim g1_web_commander` in the g1-sim image). `try`/`try_bridge` need "Take control".

## Next action

1. Find what in `isaac-view` freezes the frames: run with `--hook 'isaac-view={"ui": true}'` (camera only) vs a variant without the camera switch (hideUi only). Then add the auto-hook: `--live` on a `mujoco_isaac` suite on code path interface/simple-legacy appends `isaac-view` to `--hook` (`live_hooks(args, suite, code_path)` in `lab/cli.py` from `_resolve_run`, test in `test_tryform.py`). Push to the lab PR.
2. Seed the real-robot branch (rehearsal, GEAR kd/hands repairs, Dex3 finger gains, walk overshoot) with /handoff-brief.
3. Before merge: promote what is durable, `git rm HANDOFF.md` on all three branches, push.

**Dev env (turing):** `BRIDGE_CLONE=~/git/worktrees/humanoid_bridge/mujoco-sim BRIDGE_WS_HOST=~/.cache/g1_sim/bench_ws LAB_STORE=~/lab-store WANDB_MODE=offline PYTHONPATH=<locoman worktree> ~/github/humanoid-locoman-vla/.venv/bin/python -u -m lab try ...`; lab tests `python -m unittest discover -s lab/test` (the venv has no pytest).

## Blockers

None.

## Gotchas (unpromoted)

- `estop.require_remote` is true by default (#26) and refuses every arm without the remote header in `/lowstate`; the sim sets it. Never turn it off in `G1_config.yaml`; a sim-only params file if ever needed.
- The bridge latches damp when `/lowstate` is older than 0.2 s; safety timers use the steady clock (#16): a sim slower than real time trips it.
- Never let sim DDS reach the robot NIC: pin it to `lo`; never source `scripts/setup_unitree.sh` in a sim shell.
- Two runs at once collide on the policy server port 22085 (`port 22085 is in use`).
- Hook `on_physics_step` substeps count from the episode reset, stabilization included (160 steps = 640 substeps).
- Agent sandbox: `docker`, git writes outside the cwd worktree and pushes need the unsandboxed retry; the unsandboxed `$TMPDIR` is `/tmp`.
- Gates: bridge `tools/test.sh --docker`, commander bare `tools/test.sh --docker` with `BRIDGE_CLONE`; a skipped stage exits 2. The `--sim` stages are opt-in and host-only (g1-sim image; recipe in g1_sim `README.md`).
- `CameraServer` binds 192.168.123.164 by default (#27): a sim camera passes `bind`/`--camera-bind 127.0.0.1` (g1_sim does).
- A test commander sets `G1_WEB_COMMANDER_LOCK`; one in the g1-sim container with `$HOME` at its own path shares the host commander's lock.
- Commander in the `g1-sim` image needs `--network host`, `$HOME` and `/tmp` at their own paths, the docker socket, host docker and uv, and `LAB_STORE=~/lab-store` (the workspace's `defaults.env` points at the lab PC).
- A root container in a mounted workspace leaves root-owned files: chown them back.
