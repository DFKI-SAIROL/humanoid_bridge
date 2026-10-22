#!/usr/bin/env bash
# Runs a fresh G1_bridge's target gates against a fake robot that starts at START_Q. No hardware. Exit 1 on failure.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$HERE/loopback_dds.sh"
PKG="$(cd "$HERE/../.." && pwd)"
PARAMS="$PKG/params/G1_config.yaml"
OUT="$(mktemp -d)"
START_Q=0.2  # not 0: an unseeded rate limit would pull every joint toward 0

export PYTHONPATH="$PKG:$HERE:${PYTHONPATH:-}"

FAKE_PID=""; BRIDGE_PID=""
cleanup() {
    # Kill the process group: killing `ros2 run` alone orphans G1_bridge.
    [ -n "$BRIDGE_PID" ] && kill -9 -- "-$BRIDGE_PID" 2>/dev/null || true
    [ -n "$FAKE_PID" ] && kill -9 "$FAKE_PID" 2>/dev/null || true
}
trap cleanup EXIT

command -v ros2 >/dev/null || { echo "source the ROS workspace first"; exit 1; }

python3 "$HERE/fake_g1.py" "$OUT/lowcmd.json" --start-q "$START_Q" > "$OUT/fake.log" 2>&1 &
FAKE_PID=$!; disown "$FAKE_PID"  # no 'Killed' notice when cleanup kills it
sleep 2

setsid ros2 run robot_bridge G1_bridge --ros-args -r __node:=robot_bridge \
    --params-file "$PARAMS" > "$OUT/bridge.log" 2>&1 &
BRIDGE_PID=$!; disown "$BRIDGE_PID"  # no 'Killed' notice when cleanup kills it
sleep 4

if ! grep -q "G1 type:" "$OUT/bridge.log"; then
    echo "bridge did not come up:"; tail -20 "$OUT/bridge.log"; exit 1
fi

if ! python3 "$HERE/drive_gates.py" "$PARAMS" --start-q "$START_Q"; then
    echo; echo "bridge log, last 30 lines:"; tail -30 "$OUT/bridge.log"; exit 1
fi

# A second bridge on the same config minus its estop: block. SIGINT, not SIGKILL: a killed
# bridge's /lowcmd publisher stays in discovery, and the next bridge waits on it.
kill -INT -- "-$BRIDGE_PID" 2>/dev/null || true
for _ in $(seq 1 50); do kill -0 -- "-$BRIDGE_PID" 2>/dev/null || break; sleep 0.1; done
kill -9 -- "-$BRIDGE_PID" 2>/dev/null || true
python3 -c 'import sys, yaml
config = yaml.safe_load(open(sys.argv[1]))
del config["robot_bridge"]["ros__parameters"]["estop"]
yaml.safe_dump(config, open(sys.argv[2], "w"))' "$PARAMS" "$OUT/no_estop.yaml"
echo; echo "=== no estop: block ==="
setsid ros2 run robot_bridge G1_bridge --ros-args -r __node:=robot_bridge \
    --params-file "$OUT/no_estop.yaml" > "$OUT/bridge_no_estop.log" 2>&1 &
BRIDGE_PID=$!; disown "$BRIDGE_PID"  # no 'Killed' notice when cleanup kills it
if ! python3 "$HERE/drive_no_estop_block.py" "$OUT/bridge_no_estop.log" --start-q "$START_Q"; then
    echo; echo "bridge log, last 30 lines:"; tail -30 "$OUT/bridge_no_estop.log"; exit 1
fi
