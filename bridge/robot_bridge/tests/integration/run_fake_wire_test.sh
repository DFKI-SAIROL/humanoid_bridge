#!/usr/bin/env bash
# Runs the real G1_bridge against a fake robot. No hardware. Exit 1 on failure.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$HERE/loopback_dds.sh"
# The package root, not the repo root: this keeps working if the repo is
# rearranged around it.
PKG="$(cd "$HERE/../.." && pwd)"
PARAMS="$PKG/params/G1_config.yaml"
OUT="$(mktemp -d)"
CONFIG="${GEAR_CONFIG:-}"

export PYTHONPATH="$PKG:$HERE:${PYTHONPATH:-}"

FAKE_PID=""; BRIDGE_PID=""
cleanup() {
    # ros2 run is a launcher: killing it orphans G1_bridge, which keeps holding
    # /lowcmd and blocks the next run. Kill the whole process group.
    [ -n "$BRIDGE_PID" ] && kill -9 -- "-$BRIDGE_PID" 2>/dev/null || true
    [ -n "$FAKE_PID" ] && kill -9 "$FAKE_PID" 2>/dev/null || true
}
trap cleanup EXIT

command -v ros2 >/dev/null || { echo "source the ROS workspace first"; exit 1; }

# The fake robot first: /lowstate must exist and nothing may publish /lowcmd.
python3 "$HERE/fake_g1.py" "$OUT/lowcmd.json" > "$OUT/fake.log" 2>&1 &
FAKE_PID=$!  # not disowned: the script waits for it to write lowcmd.json
sleep 2

setsid ros2 run robot_bridge G1_bridge --ros-args -r __node:=robot_bridge \
    --params-file "$PARAMS" > "$OUT/bridge.log" 2>&1 &
BRIDGE_PID=$!; disown "$BRIDGE_PID"  # no 'Killed' notice when cleanup kills it
sleep 4

if ! grep -q "G1 type:" "$OUT/bridge.log"; then
    echo "bridge did not come up:"; tail -20 "$OUT/bridge.log"; exit 1
fi

python3 "$HERE/drive_gear_wbc.py" ${CONFIG:+--config "$CONFIG"}

sleep 1
kill -TERM "$FAKE_PID"; wait "$FAKE_PID" 2>/dev/null || true; FAKE_PID=""

echo
python3 "$HERE/check_lowcmd.py" "$OUT/lowcmd.json" --params "$PARAMS" ${CONFIG:+--config "$CONFIG"}
