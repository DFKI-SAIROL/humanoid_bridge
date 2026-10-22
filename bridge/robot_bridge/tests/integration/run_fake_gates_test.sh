#!/usr/bin/env bash
# Runs a fresh G1_bridge's target gates against a fake robot that starts at START_Q. No hardware. Exit 1 on failure.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$HERE/harness.sh"
START_Q=0.2  # not 0: an unseeded rate limit would pull every joint toward 0

FAKE_PID=""; BRIDGE_PID=""
cleanup() {
    kill_group "$BRIDGE_PID"
    kill_pid "$FAKE_PID"
}
trap cleanup EXIT

require_ros

python3 "$HERE/fake_g1.py" "$OUT/lowcmd.json" --start-q "$START_Q" > "$OUT/fake.log" 2>&1 &
FAKE_PID=$!; disown "$FAKE_PID"  # no 'Killed' notice when cleanup kills it
sleep 2

start_bridge "$OUT/bridge.log"
BRIDGE_PID=$!; disown "$BRIDGE_PID"
sleep 4
bridge_up "$OUT/bridge.log"

or_tail "$OUT/bridge.log" python3 "$HERE/drive_gates.py" "$PARAMS" --start-q "$START_Q"

# A second bridge on the same config minus its estop: block. SIGINT, not SIGKILL: a killed
# bridge's /lowcmd publisher stays in discovery, and the next bridge waits on it.
kill -INT -- "-$BRIDGE_PID" 2>/dev/null || true
for _ in $(seq 1 50); do kill -0 -- "-$BRIDGE_PID" 2>/dev/null || break; sleep 0.1; done
kill_group "$BRIDGE_PID"
python3 -c 'import sys, yaml
config = yaml.safe_load(open(sys.argv[1]))
del config["robot_bridge"]["ros__parameters"]["estop"]
yaml.safe_dump(config, open(sys.argv[2], "w"))' "$PARAMS" "$OUT/no_estop.yaml"
echo; echo "=== no estop: block ==="
start_bridge "$OUT/bridge_no_estop.log" "$OUT/no_estop.yaml"
BRIDGE_PID=$!; disown "$BRIDGE_PID"
or_tail "$OUT/bridge_no_estop.log" python3 "$HERE/drive_no_estop_block.py" "$OUT/bridge_no_estop.log" --start-q "$START_Q"
