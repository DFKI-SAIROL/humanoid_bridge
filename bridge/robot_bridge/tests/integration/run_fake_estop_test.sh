#!/usr/bin/env bash
# Runs the real G1_bridge's e-stop against a fake robot. No hardware. Exit 1 on failure.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG="$(cd "$HERE/../.." && pwd)"
PARAMS="$PKG/params/G1_config.yaml"
OUT="$(mktemp -d)"

export PYTHONPATH="$PKG:$HERE:${PYTHONPATH:-}"

FAKE_PID=""; BRIDGE_PID=""
cleanup() {
    # Kill the process group: killing `ros2 run` alone orphans G1_bridge.
    [ -n "$BRIDGE_PID" ] && kill -9 -- "-$BRIDGE_PID" 2>/dev/null || true
    [ -n "$FAKE_PID" ] && kill -9 "$FAKE_PID" 2>/dev/null || true
}
trap cleanup EXIT

command -v ros2 >/dev/null || { echo "source the ROS workspace first"; exit 1; }

python3 "$HERE/fake_g1.py" "$OUT/lowcmd.json" > "$OUT/fake.log" 2>&1 &
FAKE_PID=$!; disown "$FAKE_PID"  # no 'Killed' notice when cleanup kills it
sleep 2

setsid ros2 run robot_bridge G1_bridge --ros-args -r __node:=robot_bridge \
    --params-file "$PARAMS" > "$OUT/bridge.log" 2>&1 &
BRIDGE_PID=$!; disown "$BRIDGE_PID"  # no 'Killed' notice when cleanup kills it
sleep 4

if ! grep -q "G1 type:" "$OUT/bridge.log"; then
    echo "bridge did not come up:"; tail -20 "$OUT/bridge.log"; exit 1
fi

if ! python3 "$HERE/drive_estop.py" "$PARAMS"; then
    echo; echo "bridge log, last 30 lines:"; tail -30 "$OUT/bridge.log"; exit 1
fi
