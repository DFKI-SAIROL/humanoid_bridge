#!/usr/bin/env bash
# Runs the real G1_bridge against a fake robot. No hardware. Exit 1 on failure.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$HERE/harness.sh"
CONFIG="${GEAR_CONFIG:-}"

FAKE_PID=""; BRIDGE_PID=""
cleanup() {
    kill_group "$BRIDGE_PID"
    kill_pid "$FAKE_PID"
}
trap cleanup EXIT

require_ros

# The fake robot first: /lowstate must exist and nothing may publish /lowcmd.
python3 "$HERE/fake_g1.py" "$OUT/lowcmd.json" > "$OUT/fake.log" 2>&1 &
FAKE_PID=$!  # not disowned: the script waits for it to write lowcmd.json
sleep 2

start_bridge "$OUT/bridge.log"
BRIDGE_PID=$!; disown "$BRIDGE_PID"  # no 'Killed' notice when cleanup kills it
sleep 4
bridge_up "$OUT/bridge.log"

python3 "$HERE/drive_gear_wbc.py" ${CONFIG:+--config "$CONFIG"}

sleep 1
kill -TERM "$FAKE_PID"; wait "$FAKE_PID" 2>/dev/null || true; FAKE_PID=""

echo
python3 "$HERE/check_lowcmd.py" "$OUT/lowcmd.json" --params "$PARAMS" ${CONFIG:+--config "$CONFIG"}
