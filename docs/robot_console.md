# Robot console

One web page to operate a robot through the bridge: e-stop, arm, start and
stop processes, jog motors, watch the camera. Package `bridge/robot_console`,
robot-generic, upstream-bound. Decided with the author 2026-10-19.

Status: implemented on `g1-web-interface`; unit and fake-robot tests pass; gantry checks pending.

## Shape

| | |
|---|---|
| Runs on | the lab PC, beside the bridge, in its own tmux session |
| Start | `ros2 run robot_console robot_console --config <file> [--host <lan-ip>] [--port 8080]` |
| Stack | stdlib `http.server` + RFC 6455 WebSocket, `rclpy`, PyYAML. No new deps |
| Code base | `motor_probe` from `feat-motor-probe` (`03451f9`), without its `vendor/` |
| Bridge | stays in tmux (`tools/g1-bringup.sh`); the console reads `/bridge_state`, calls services |
| Lab specifics | none in code: one YAML per setup, the lab's lives in `humanoid-locoman-vla/benches/g1/console.yaml` |

## Access

- Token in `~/.config/robot_console/token` (mode 600), made on first start, kept across restarts.
- `/?token=<t>` sets an `HttpOnly; SameSite=Strict` cookie; every other route needs it.
- POST and WebSocket: an `Origin` that does not match `Host` is refused.
- Binds `--host` (default `127.0.0.1`). `0.0.0.0`/`::` are refused: bind the lab-LAN address, so the robot network cannot reach it.
- Plain HTTP: anyone who sniffs the lab LAN can read the token.

## Safety rules

| Rule | Why |
|---|---|
| E-STOP DAMP / HOLD: any tab, one bridge call from the web thread, never queued | must not wait behind a slow service call |
| The remote (`L1+R1`, `R1+R2`) is the primary e-stop; a browser disconnect does nothing to processes | a sleeping laptop must not end a run |
| One controlling tab: arm, ready, reset, process start, prompt answers, jog | two people jogging unknowingly |
| Any tab: e-stop, `/stop_control`, process stop | stopping is the safe direction |
| Arm and ready need a confirm dialog | same as `--robot-is-clear` |
| Arm refused while a `commands_robot` process or a jog runs | a live stream cuts the arming ramp short |
| Jog and `commands_robot` processes exclude each other | both publish `/robot_cmd` |
| `requires` is enforced: Start stays disabled until every condition holds | arm before GEAR, GEAR before the node |
| Child processes die with the console | a run without a page that shows it |
| Jog deadman: no browser ping for 0.5 s stops the jog stream; the bridge then latches `hold` (`cmd_timeout`) | closing the tab stops the robot |
| Ending a jog calls `/stop_control` first | a clean end must not latch |
| The jog command slews toward the slider at the chosen max speed (0.2 / 0.5 / 1.0 rad/s, default 0.5) and is sent at 50 Hz with a 40 ms `duration` | a click on the slider is a ramp, not a jump |

The latch and `/bridge_state` are valid on the G1 only; on other robots the
safety bar says so.

## Config — one file, two sections

```yaml
console:
  robot: G1
  bridge_config: ${BRIDGE_CLONE}/bridge/robot_bridge/params/G1_config.yaml   # motor catalog
  lowstate: {topic: /lowstate, type: unitree_hg/msg/LowState}
  hands: true                  # Dex3 jog and hand state
  urdf: ~/.local/share/robot_console/g1_mjlab/g1.urdf                       # optional, 3D view
  camera: http://192.168.123.164:5557/frame.jpg                             # optional, proxied
  log_dir: ~/.local/state/robot_console

processes:
  <name>:
    label: GEAR WBC
    group: policy                 # page section: bring-up | policy | run
    cwd: ~/github/humanoid-locoman-vla
    env_file: .env                # KEY=VALUE, relative to cwd; the real environment wins
    env: {CONTAINER_NAME: console-gear}   # wins over both
    argv: [docker/bench-g1.sh, bash, -c, "…"]
    params:                       # "{name}" in argv
      ckpt: {choices: [task3]}
      instruction: {text: true, pattern: "^[ -~]{0,200}$"}
    tty: false                    # true: a pty (ssh -tt, stdin prompts)
    ready: {http: "http://127.0.0.1:22085/health"}   # or {log: "regex"}
    stop: {signals: [INT, TERM, KILL], grace_s: [10, 5]}   # CTRL-C = write \x03 to the pty
    stop_argv: [docker, rm, -f, console-gear]
    commands_robot: true
    requires: [armed, psi0.ready, camera.ready]      # armed | <name>.running | <name>.ready
    prompts: [{match: "ready\\? \\[go/abort\\]", answers: [go, abort]}]
```

- `${VAR}` and `~` expand in `cwd`, `argv`, `env` and `console` paths, from the merged environment, before params.
- `${ROBOT_CONSOLE_CONFIG_DIR}` is the config file's directory, so a lab file can name its own repo.
- A `choices` value may fill `{name}` anywhere in an element. A `text` value must be a whole element: never inside `bash -c`.
- `argv` runs without a shell.
- `ready` answering before Start: the entry shows `external`; it cannot be started or stopped here.

## Processes

| State | Meaning |
|---|---|
| `stopped` | never started, or ended by Stop |
| `starting` | running, `ready` not yet true |
| `ready` | running and `ready` true (no `ready` key: running counts as ready) |
| `prompt` | a `prompts` pattern matched the output; waiting for an answer |
| `stopping` | the stop sequence is running |
| `exited` | ended by itself; exit code shown |
| `external` | `ready` answers and the console did not start it |

- Spawned from the supervisor's own thread, in a new session, under `setpriv --pdeathsig TERM`.
- Stop: the signal list to the process group with its graces, then `stop_argv`.
- Log per start: `<log_dir>/<date>/<name>-<HHMMSS>.log`; the page tails the last 200 lines.

## HTTP API

| Route | Who | Does |
|---|---|---|
| `GET /api/stream` (WebSocket) | any | `hello` with the tab's client id, then a snapshot every 0.1 s; the tab sends `ping` every 250 ms |
| `GET /api/catalog` | any | the motor table |
| `GET /api/process/log?name=&since=` | any | log lines after `since` |
| `GET /api/camera/frame.jpg` | any | the configured camera URL, proxied |
| `POST /api/estop` `{mode}` | any | `/estop_damp` or `/estop_hold` |
| `POST /api/stop` | any | `/stop_control` |
| `POST /api/process/stop` `{name}` | any | stop sequence |
| `POST /api/control/take` | any | this tab becomes the controller |
| `POST /api/reset` | controller | `/estop_reset` |
| `POST /api/arm` `{confirm}` | controller | `start_control` at the measured pose |
| `POST /api/ready` `{confirm}` | controller | `/ready_position_control` |
| `POST /api/process/start` `{name, params}` | controller | start, after `requires` |
| `POST /api/process/answer` `{name, answer}` | controller | answer + newline to the pty |
| `POST /api/jog/{start,stop,select,target,mode}` | controller | the motor jog |

Every POST carries `client` (from `hello`); the controller is the tab whose
WebSocket holds the lock. The lock frees when that socket closes.

## Panels

| Panel | Content |
|---|---|
| Safety bar (every view, fixed on a phone) | state, E-STOP DAMP, HOLD, RESET, control lock; `Esc` = damp |
| Control | arm, ready, stop; `/lowstate` age, roll/pitch |
| Processes | one card per entry: params, Start/Stop, state, missing `requires`, prompt buttons, log tail |
| Jog | the motor probe: list, slider, limp, max speed, 3D view; its corner tag names the model (`SOURCE` beside the URDF) |
| Camera | the proxied JPEG at ~5 Hz |

## Tests

| Level | What |
|---|---|
| unit (`tools/test.sh --unit`) | config, catalog, auth, session, service rules, supervisor (real child processes), server routes, one session against `devserver`'s simulated bridge |
| `tools/test.sh --console` | the real console + `G1_bridge` + `fake_g1.py` + two dummy processes: control lock, arm, `requires`, jog on `/lowcmd`, web e-stop, reset, tab-closed deadman, remote combo, shutdown stops children |
| gantry | the e-stop gantry list, plus: phone over LAN, arm, jog one joint, observe-only run from the page |

## Run it

```bash
bridge/robot_console/scripts/fetch_assets.sh            # once per machine: three.js + mjlab G1 (--model unitree: Dex3 hands)
ros2 run robot_console robot_console --config <file> --host <lan-ip>   # in its own tmux pane
python3 -m robot_console.devserver                      # the page against a simulated bridge, no ROS
```

## Traps

- `PR_SET_PDEATHSIG` fires when the forking *thread* exits: spawn only from the supervisor's long-lived thread, never from an HTTP handler thread.
- `ssh -t` needs a terminal on stdin: use `ssh -tt` under `tty: true`. SIGINT to the local ssh kills the session without a remote Ctrl-C: stop with `CTRL-C`.
- The node and the camera server stop cleanly only on SIGINT; SIGTERM skips the node's hold goals.
- Killing `docker run` leaves the container: give it a name and a `stop_argv`.
- The lab's stdin prompts end without a newline: prompts match the unfinished last line.
- `spin_once` returns on every `/lowstate` (500 Hz): the jog paces its own publish, or it floods `/robot_cmd`.
- Start the console in a foreground tmux pane: a script's background job inherits SIGINT ignored, and its children then ignore the INT stop step.
- tty children need the pty as controlling terminal, or a written `^C` does nothing: the supervisor wraps them in a `TIOCSCTTY` shim.
