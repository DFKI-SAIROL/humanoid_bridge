# Fake wire tests

Runs the real `G1_bridge` against a fake robot on one machine. No hardware.

Covers the whole path: adapter to client to `/robot_cmd` to bridge to `/lowcmd`.
It does **not** cover the Foxy/Python 3.8 Jetson, the cable, or real motors.

## Requirements

```bash
source /opt/ros/humble/setup.bash && tools/build.sh
source <workspace>/install/setup.bash
```

`robot_bridge`'s `package.xml` depends on `booster_interface` unconditionally,
so it builds even with T1 off.

`tools/test.sh` runs both scripts below and handles the DDS setting they need.

## 1. Run

```bash
tools/test.sh --wire
```

Uses a synthetic identity config. To use GEAR's own:

```bash
GEAR_CONFIG=<path>/g1_29dof_gear_wbc.yaml ./bridge/robot_bridge/tests/integration/run_fake_wire_test.sh
```

## 2. Verify

```
received /lowcmd             PASS  4746 samples
publishes at ~1 kHz          PASS  1000 Hz
start_control honoured       PASS  q[0]=0.1230 wanted 0.123
tracked GEAR's gains         PASS  2231 samples
joint j lands on motor j     PASS  q[:3]=[0.01, 0.02, 0.03]
kp from config               PASS
kd from config               PASS
bridge forced mode 1         PASS
bridge forced PR mode        PASS
timeout latches hold         PASS  kp[0]=100.0 q[0]=0.0100
```

E-stop, ends `all checks passed` (design: `docs/estop.md`):

```bash
tools/test.sh --estop
```

## 3. The G1 hand path

```bash
tools/test.sh --hands
tools/test.sh --hands --stale-after 3.5
```

`--stale-after` silences the left hand mid-stream, to prove the release is
per-side. The clock starts at the first `/dex3/left/cmd`, which is
`start_hand_control`, so it must land between 3.0 and 4.0 s: earlier and the
left hand never tracks, later and the driver has already stopped.

`fake_g1.py` runs too, because the bridge blocks in its constructor until
something publishes `/lowstate`. That doubles as the check that the hand path
left the body idle.

## Parts

| File | Role |
|---|---|
| `fake_g1.py` | publishes `/lowstate` at 500 Hz, records `/lowcmd`, tracks commands perfectly |
| `drive_gear_wbc.py` | `start_control`, then 100 commands at 50 Hz through the adapter |
| `check_lowcmd.py` | asserts the recording, exit 1 on failure |
| `drive_estop.py` | starts before the bridge (handshake: `driver: holding /lowcmd`) and holds `/lowcmd` until the bridge log shows it was seen and no `/start_control` exists; then drives every e-stop path and checks `/lowcmd` and `/bridge_state` live; `fake_g1.py` takes `/fake_g1/keys` and `/fake_g1/roll` |
| `drive_shutdown.py` | runs last in the e-stop run: arms, SIGINTs the bridge's process group, checks damp frames for 1 s, then silence |
| `run_fake_estop_test.sh` | orchestrates the e-stop run; then SIGINT to a second bridge in its `/lowcmd` wait: exit 0 within 5 s |
| `run_fake_wire_test.sh` | orchestrates the three, cleans up |
| `fake_dex3.py` | publishes `/dex3/*/state` at 100 Hz, records `/dex3/*/cmd`; `--stale-after`, `--hot-after`, both timed from the first left command |
| `drive_hand_cmd.py` | `start_hand_control` refused with another `/lowcmd` publisher and on a tilt, then accepted, then both hands through the hand adapter, then a bad command |
| `check_hand_cmd.py` | asserts the hand recording; pure offline, needs no ROS |
| `run_fake_hand_test.sh` | orchestrates the hand run |
| `run_config_test.sh`, `drive_config.py` | `G1_bridge` on broken copies of `G1_config.yaml` (`ready_q` short or out of range, `idx` out of order, inverted limits): each must fail its parameter load. No fake robot |

## What breaks it

- `Detected 1 publishers on /lowcmd`: an orphaned `G1_bridge` from an earlier
  run. `pkill -9 -x G1_bridge`. The script starts the bridge with `setsid` and
  kills the process group, because `ros2 run` is a launcher and killing it
  alone orphans the child.
- `start_control` refused in a fake run: `fake_g1.py` lost its remote header
  (`/fake_g1/remote`), or another `/lowcmd` publisher is up.
- A wire recording ends in hold frames: GEAR's stream stops, the command
  timeout latches `hold` at the bridge's gains. `check_lowcmd.py` takes the last
  frame with GEAR's kp as the last tracking frame.
- Run with the robot cabled: `unshare -rn` gives a namespace with only `lo`
  (`ip link set lo up multicast on; ip route add 224.0.0.0/4 dev lo` inside).
- `bridge did not come up`: `/lowstate` was missing, or another node held
  `/lowcmd`. The bridge log is printed on failure.
- The fake publishes 35 motor states, the bridge is configured for 29, so a
  one-time `LowState carries 35 motor states` warning is expected.
- `hand path did not load`: the config has no `hand_*` keys. The bridge warns
  and runs the body path anyway, which is why the script greps for
  `Dex3 hand path ready` rather than trusting a clean start.
- Every hand run ends with both hands limp: the watchdog fires once the driver
  stops. `check_hand_cmd.py` asserts against the last *commanded* frame, and a
  new check that reads `samples[-1]` will be reading the release.
- CycloneDDS with no `CYCLONEDDS_URI`: multicast is off on loopback, the
  bridge never sees the fake and prints `Waiting for publisher on /lowstate`.
  `tools/test.sh` sets the URI itself; `--rmw fastrtps` is the other way out.
