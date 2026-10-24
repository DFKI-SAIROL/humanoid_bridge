#!/usr/bin/env bash
# Runs the real G1_bridge's /robot_link against a fake robot that drops its link and reboots.
# Two bridges: a restart is final for a bridge. No hardware. Exit 1 on failure.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$HERE/harness.sh"

FAKE_PID=""; BRIDGE_PID=""
cleanup() {
    kill_group "$BRIDGE_PID"
    kill_pid "$FAKE_PID"
}
trap cleanup EXIT

require_ros

python3 "$HERE/fake_g1.py" "$OUT/lowcmd.json" > "$OUT/fake.log" 2>&1 &
FAKE_PID=$!; disown "$FAKE_PID"  # no 'Killed' notice when cleanup kills it
sleep 2

start_bridge "$OUT/bridge.log"
BRIDGE_PID=$!; disown "$BRIDGE_PID"
sleep 4
bridge_up "$OUT/bridge.log"
or_tail "$OUT/bridge.log" python3 "$HERE/drive_link.py" armed

# SIGINT, not SIGKILL: a killed bridge's /lowcmd publisher stays in discovery, and the next bridge waits on it.
kill -INT -- "-$BRIDGE_PID" 2>/dev/null || true
for _ in $(seq 1 50); do kill -0 -- "-$BRIDGE_PID" 2>/dev/null || break; sleep 0.1; done
kill_group "$BRIDGE_PID"

echo; echo "=== a fresh bridge, disarmed reboot ==="
start_bridge "$OUT/bridge2.log"
BRIDGE_PID=$!; disown "$BRIDGE_PID"
sleep 4
bridge_up "$OUT/bridge2.log"
or_tail "$OUT/bridge2.log" python3 "$HERE/drive_link.py" disarmed
