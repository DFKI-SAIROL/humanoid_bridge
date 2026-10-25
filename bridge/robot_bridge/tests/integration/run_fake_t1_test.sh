#!/usr/bin/env bash
# Runs the real T1_bridge against fake_t1: /robot_state, custom mode on arming, the gamepad
# e-stop, a /low_state longer than the config, damping mode on SIGINT. No hardware. Exit 1 on failure.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$HERE/harness.sh"
T1_PARAMS="$PKG/params/T1_config.yaml"

FAKE_PID=""; BRIDGE_PID=""
cleanup() {
    kill_group "$BRIDGE_PID"
    kill_pid "$FAKE_PID"
}
trap cleanup EXIT

require_ros

python3 "$HERE/fake_t1.py" --start-q 0.05 > "$OUT/fake.log" 2>&1 &
FAKE_PID=$!; disown "$FAKE_PID"
sleep 2

setsid ros2 run robot_bridge T1_bridge --ros-args -r __node:=robot_bridge \
    --params-file "$T1_PARAMS" > "$OUT/bridge.log" 2>&1 &
BRIDGE_PID=$!; disown "$BRIDGE_PID"
sleep 4
if ! grep -q "T1 type:" "$OUT/bridge.log"; then
    echo "bridge did not come up:"; tail -20 "$OUT/bridge.log"; exit 1
fi
or_tail "$OUT/bridge.log" python3 "$HERE/drive_t1.py" "$T1_PARAMS"

echo
if grep -q "LowState carries 25 serial motor states" "$OUT/bridge.log"; then
    verdict "a 25-motor /low_state is bounded and logged" PASS
else
    verdict "a 25-motor /low_state is bounded and logged" FAIL; tail -20 "$OUT/bridge.log"; exit 1
fi
kill -INT -- "-$BRIDGE_PID" 2>/dev/null || true
for _ in $(seq 1 50); do kill -0 -- "-$BRIDGE_PID" 2>/dev/null || break; sleep 0.1; done
if grep -q 'change mode {"mode":0}' "$OUT/fake.log"; then
    verdict "damping mode on SIGINT" PASS
else
    verdict "damping mode on SIGINT" FAIL; tail -20 "$OUT/fake.log"; tail -20 "$OUT/bridge.log"; exit 1
fi
echo "all checks passed"
