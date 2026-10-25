#!/usr/bin/env bash
# Runs the real G1_bridge against robot_sim (MuJoCo). No hardware. Exit 1 on failure.
# Needs: the workspace with robot_bridge and robot_sim sourced, and the mjlab g1.xml ($G1_MJCF).
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../../../../.." && pwd)"
# shellcheck disable=SC1091
source "$REPO/bridge/robot_bridge/tests/integration/loopback_dds.sh"
PARAMS="$REPO/bridge/robot_bridge/params/G1_config.yaml"
OUT="$(mktemp -d)"

export PYTHONPATH="$REPO/bridge/robot_bridge:${PYTHONPATH:-}"

SIM_PID=""; BRIDGE_PID=""
cleanup() {
    # Kill process groups: killing `ros2 run` alone orphans its child.
    [ -n "$BRIDGE_PID" ] && kill -9 -- "-$BRIDGE_PID" 2>/dev/null || true
    [ -n "$SIM_PID" ] && kill -9 -- "-$SIM_PID" 2>/dev/null || true
}
trap cleanup EXIT

command -v ros2 >/dev/null || { echo "source the ROS workspace first"; exit 1; }

setsid ros2 run robot_sim robot_sim --robot g1 --no-viewer > "$OUT/sim.log" 2>&1 &
SIM_PID=$!; disown "$SIM_PID"  # no 'Killed' notice when cleanup kills it
sleep 3
grep -q "gantry on" "$OUT/sim.log" || { echo "sim did not come up:"; tail -20 "$OUT/sim.log"; exit 1; }

setsid ros2 run robot_bridge G1_bridge --ros-args -r __node:=robot_bridge \
    --params-file "$PARAMS" > "$OUT/bridge.log" 2>&1 &
BRIDGE_PID=$!; disown "$BRIDGE_PID"  # no 'Killed' notice when cleanup kills it
sleep 4
grep -q "G1 type:" "$OUT/bridge.log" || { echo "bridge did not come up:"; tail -20 "$OUT/bridge.log"; exit 1; }

if ! python3 "$HERE/drive_sim.py" "$PARAMS"; then
    echo; echo "bridge log, last 30 lines:"; tail -30 "$OUT/bridge.log"
    echo; echo "sim log, last 10 lines:"; tail -10 "$OUT/sim.log"; exit 1
fi
# No target here leaves its range: a clamp is a seed or ramp bug (robot_sim rests waist pitch past q_max).
if grep "outside \[" "$OUT/bridge.log"; then
    echo "FAIL  the bridge clamped a target"; exit 1
fi
if grep -q "behind real time" "$OUT/sim.log"; then
    echo "WARN: the sim fell behind real time:"; grep "behind real time" "$OUT/sim.log" | tail -3
fi
