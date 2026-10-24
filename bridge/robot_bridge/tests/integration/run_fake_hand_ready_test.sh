#!/usr/bin/env bash
# Runs the real G1_bridge's Ready against fake Dex3 hands: Ready opens every hand with state.
# No hardware. Exit 1 on failure.
#
#   run_fake_hand_ready_test.sh [--silent left|right]
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$HERE/harness.sh"

SILENT=""
case "${1:-}" in
    --silent) SILENT="${2:?--silent needs left or right}" ;;
    "") ;;
    *) echo "usage: $0 [--silent left|right]"; exit 1 ;;
esac

FAKE_PID=""; HAND_PID=""; BRIDGE_PID=""
cleanup() {
    kill_group "$BRIDGE_PID"
    kill_pid "$HAND_PID"
    kill_pid "$FAKE_PID"
}
trap cleanup EXIT

require_ros

python3 "$HERE/fake_g1.py" "$OUT/lowcmd.json" > "$OUT/fake_g1.log" 2>&1 &
FAKE_PID=$!
python3 "$HERE/fake_dex3.py" "$OUT/handcmd.json" ${SILENT:+--silent "$SILENT"} \
    > "$OUT/fake_dex3.log" 2>&1 &
HAND_PID=$!
sleep 2

start_bridge "$OUT/bridge.log"
BRIDGE_PID=$!
sleep 4
bridge_up "$OUT/bridge.log"
if ! grep -q "Dex3 hand path ready" "$OUT/bridge.log"; then
    echo "hand path did not load; check the hand_* parameters:"
    grep -i hand "$OUT/bridge.log" | tail -20; exit 1
fi

or_tail "$OUT/bridge.log" python3 "$HERE/drive_hand_ready.py" "$PARAMS" ${SILENT:+--silent "$SILENT"}
