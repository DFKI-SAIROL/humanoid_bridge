#!/usr/bin/env bash
# Runs robot_console against the real G1_bridge and a fake robot. No hardware. Exit 1 on failure.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG="$(cd "$HERE/../.." && pwd)"
PARAMS="$PKG/params/G1_config.yaml"
OUT="$(mktemp -d)"
PORT="$(python3 -c 'import socket; s = socket.socket(); s.bind(("127.0.0.1", 0)); print(s.getsockname()[1])')"
MARK="console-test-marker-$$"

export PYTHONPATH="$PKG:$HERE:${PYTHONPATH:-}"

FAKE_PID=""; BRIDGE_PID=""; CONSOLE_PID=""
cleanup() {
    # Kill process groups: `ros2 run` is a launcher, killing it alone orphans the child.
    [ -n "$CONSOLE_PID" ] && kill -9 -- "-$CONSOLE_PID" 2>/dev/null || true
    [ -n "$BRIDGE_PID" ] && kill -9 -- "-$BRIDGE_PID" 2>/dev/null || true
    [ -n "$FAKE_PID" ] && kill -9 "$FAKE_PID" 2>/dev/null || true
    pkill -9 -f "$MARK" 2>/dev/null || true
}
trap cleanup EXIT

command -v ros2 >/dev/null || { echo "source the ROS workspace first"; exit 1; }
ros2 pkg prefix robot_console >/dev/null 2>&1 || { echo "robot_console is not built: tools/build.sh"; exit 1; }

cat > "$OUT/console.yaml" <<EOF
console:
  robot: G1
  bridge_config: $PARAMS
  lowstate: {topic: /lowstate, type: unitree_hg/msg/LowState}
  hands: false
  log_dir: $OUT/logs
processes:
  marker:
    group: bring-up
    argv: [python3, -u, -c, "import time; print('ready $MARK', flush=True); time.sleep(600)"]
    ready: {log: "ready $MARK"}
  streamer:
    group: policy
    commands_robot: true
    requires: [armed, marker.ready]
    argv: [python3, -u, -c, "import time; print('{mode} $MARK', flush=True); time.sleep(600)"]
    params: {mode: {choices: [slow, fast]}}
EOF

python3 "$HERE/fake_g1.py" "$OUT/lowcmd.json" > "$OUT/fake.log" 2>&1 &
FAKE_PID=$!; disown "$FAKE_PID"  # no 'Killed' notice when cleanup kills it
sleep 2

setsid ros2 run robot_bridge G1_bridge --ros-args -r __node:=robot_bridge \
    --params-file "$PARAMS" > "$OUT/bridge.log" 2>&1 &
BRIDGE_PID=$!; disown "$BRIDGE_PID"  # no 'Killed' notice when cleanup kills it
sleep 4
grep -q "G1 type:" "$OUT/bridge.log" || { echo "bridge did not come up:"; tail -20 "$OUT/bridge.log"; exit 1; }

setsid ros2 run robot_console robot_console --config "$OUT/console.yaml" --port "$PORT" \
    --token-file "$OUT/token" --vendor "$OUT/vendor" > "$OUT/console.log" 2>&1 &
CONSOLE_PID=$!; disown "$CONSOLE_PID"  # no 'Killed' notice when cleanup kills it
for _ in $(seq 1 40); do
    grep -q "robot console on" "$OUT/console.log" 2>/dev/null && break
    sleep 0.25
done
grep -q "robot console on" "$OUT/console.log" || { echo "console did not come up:"; tail -20 "$OUT/console.log"; exit 1; }

if ! python3 "$HERE/drive_console.py" "$PORT" "$OUT/token" "$PARAMS"; then
    echo; echo "console log, last 30 lines:"; tail -30 "$OUT/console.log"; exit 1
fi

# Ctrl-C on the console must take its processes down with it.
kill -INT -- "-$CONSOLE_PID"
for _ in $(seq 1 60); do
    pgrep -f "$MARK" >/dev/null || break
    sleep 0.25
done
if pgrep -f "$MARK" >/dev/null; then
    echo "console shutdown left its processes running         FAIL"; exit 1
fi
echo "console shutdown stops its processes                PASS"
