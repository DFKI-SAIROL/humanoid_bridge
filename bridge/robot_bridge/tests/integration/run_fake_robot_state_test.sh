#!/usr/bin/env bash
# Checks the real G1_bridge's /robot_state and /hand_state against fake_g1 and fake_dex3. No hardware.
# Exit 1 on failure.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$HERE/harness.sh"

FAKE_PID=""; HAND_PID=""; BRIDGE_PID=""
cleanup() {
    kill_group "$BRIDGE_PID"
    kill_pid "$HAND_PID"
    kill_pid "$FAKE_PID"
}
trap cleanup EXIT

require_ros

START_Q=0.1
python3 "$HERE/fake_g1.py" "$OUT/lowcmd.json" --start-q "$START_Q" > "$OUT/fake_g1.log" 2>&1 &
FAKE_PID=$!; disown "$FAKE_PID"
python3 "$HERE/fake_dex3.py" "$OUT/handcmd.json" > "$OUT/fake_dex3.log" 2>&1 &
HAND_PID=$!; disown "$HAND_PID"
sleep 2

start_bridge "$OUT/bridge.log"
BRIDGE_PID=$!; disown "$BRIDGE_PID"
sleep 4
bridge_up "$OUT/bridge.log"
or_tail "$OUT/bridge.log" python3 "$HERE/drive_robot_state.py" "$PARAMS" "$START_Q"
