# g1_siwei: probing motion controller on the G1

Everything runs on the robot (unitree@192.168.123.164, Ubuntu 20.04 / ROS Foxy / Python 3.8, aarch64,
no internet, clock at 1970).

| Path | What |
|---|---|
| `deploy/TI-B_inclHO_seed0/` | Deploy bundle from g1_raps `g1_deploy` (2944bbb): policy.onnx, deploy.yaml, 66 motion packs, held stand, parity recordings. `sha256sum -c SHA256SUMS` passes here. |
| `ws/src/humanoid_bridge/` | humanoid_bridge branch `g1_probe_client` (B-1 state-write fix + the probe client in `robot_bridge/example/G1_probe/`); built in `ws/` (separate from `~/sairol_ws`). |
| `pylib/` | numpy 1.23.5 + onnxruntime 1.16.3 aarch64 wheels (system numpy 1.17.4 is too old for onnxruntime); only on PYTHONPATH via `env.sh`. |
| `wheels/` | The wheels `pylib/` was installed from (`pip3 install --no-index --no-deps --target pylib wheels/*.whl`). |
| `env.sh` | ROS Foxy + cyclonedds (eth0) + pylib + this workspace; numpy/OpenMP thread pools pinned to 1 thread. |
| `start.sh` / `stop.sh` | Start / stop bridge + client, detached from the shell (they survive SSH or cable drops). |
| `logs/` | bridge.log (ends with the bridge's exit code), client.log. |
| `probe_logs/runs/` | One npz per probing motion: first row to end (or through the damping ramp if damped). Per tick: obs, action, target, kp, q, dq, tau_est, quat, gyro, ref_row, q_ref, state_age, lowstate_tick; `meta_json` has motion, outcome, hashes. |
| `probe_logs/sessions/` | Everything while the policy runs, in 5 min chunks; written when the bridge is stopped. |

## Run

```bash
ssh unitree@192.168.123.164        # or unitree@10.42.0.1 over the robot hotspot (SSID G1-Onboard)
~/g1_siwei/start.sh
```

Open the web commander at http://10.42.0.1:8080/ (hotspot G1-Onboard, preferred) or
http://192.168.123.164:8080/ (cable). L2+R2 for debug mode (robot on the gantry: it goes limp), then
HOME, Stand, motions. The **Data** tab lists the recordings, downloads them to the browser's machine
(one file, or a zip) and deletes them from the robot. The page listens on all interfaces: anyone on
the robot's network or hotspot can command it.

Stops: DAMPING button / space / remote L2+B (1 s ramp to damping; works while the policy runs and
while the bridge holds HOME); remote L2+UP+LEFT (bridge stops sending commands). `~/g1_siwei/stop.sh`
damps if needed, stops the bridge, saves, and ends both processes. Press **Stop bridge** before
power-cycling the robot, or the session data still in memory is lost.

Clicks that reach the robot more than 1 s after they were made (dropped link) are not executed;
DAMPING and Stop always are.

## Checks

```bash
source ~/g1_siwei/env.sh
cd $G1_PROBE && python3 test_offline.py --bundle $G1_BUNDLE       # client step vs sim parity
python3 G1_probe.py --bundle $G1_BUNDLE --shadow --web-port 0     # live state, publishes nothing
```

The robot clock is at 1970: `make` warns about clock skew, so after editing C++ do a clean build
(`rm -rf build/robot_bridge install/robot_bridge && colcon build --packages-select robot_bridge --cmake-args -DBUILD_BOOSTER_T1=OFF`).
