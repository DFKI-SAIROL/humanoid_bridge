#!/usr/bin/env bash
# G1_bridge's /robot_link against robot_sim: link drop, reboot, then the restart bring-up through
# tools/g1-bringup.sh --sim, which releases the sim's motion service with unitree_sdk2py.
# Needs: the workspace with robot_bridge and robot_sim sourced, $G1_MJCF, unitree_sdk2py (the robot-sim image).
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../../../../.." && pwd)"
INTEGRATION="$REPO/bridge/robot_bridge/tests/integration"
# shellcheck disable=SC1091
source "$INTEGRATION/loopback_dds.sh"
OUT="$(mktemp -d)"
export PYTHONPATH="$REPO/bridge/robot_bridge:$INTEGRATION:${PYTHONPATH:-}"

SIM_PID=""; BRIDGE_PID=""
cleanup() {
    # Kill process groups: killing `ros2 run` alone orphans its child.
    [ -n "$BRIDGE_PID" ] && kill -9 -- "-$BRIDGE_PID" 2>/dev/null || true
    [ -n "$SIM_PID" ] && kill -9 -- "-$SIM_PID" 2>/dev/null || true
}
trap cleanup EXIT

command -v ros2 >/dev/null || { echo "source the ROS workspace first"; exit 1; }
PREFIX="$(ros2 pkg prefix robot_bridge)"
WS="${PREFIX%/install/robot_bridge}"

# bring_up <log>: g1-bringup.sh --sim --foreground in its own process group; exit 1 unless the bridge comes up.
bring_up() {
    setsid "$REPO/tools/g1-bringup.sh" --sim --foreground --workspace "$WS" > "$1" 2>&1 &
    BRIDGE_PID=$!; disown "$BRIDGE_PID"
    for _ in $(seq 1 60); do grep -q "G1 type:" "$1" && return 0; sleep 0.5; done
    echo "bring-up did not come up:"; tail -30 "$1"; exit 1
}
or_tail() {   # or_tail <log> <command...>
    local log="$1"; shift
    if ! "$@"; then echo; echo "bring-up log, last 30 lines:"; tail -30 "$log"; exit 1; fi
}

setsid ros2 run robot_sim robot_sim --robot g1 --no-viewer --no-camera > "$OUT/sim.log" 2>&1 &
SIM_PID=$!; disown "$SIM_PID"
sleep 3
grep -q "gantry on" "$OUT/sim.log" || { echo "sim did not come up:"; tail -20 "$OUT/sim.log"; exit 1; }

bring_up "$OUT/bringup.log"
or_tail "$OUT/bringup.log" python3 "$HERE/drive_sim_link.py" restart

# What the commander's "Restart bring-up" does: stop the bridge, bring it up again; the start releases.
kill -INT -- "-$BRIDGE_PID" 2>/dev/null || true
for _ in $(seq 1 50); do kill -0 -- "-$BRIDGE_PID" 2>/dev/null || break; sleep 0.1; done
kill -9 -- "-$BRIDGE_PID" 2>/dev/null || true

echo; echo "=== restart bring-up ==="
bring_up "$OUT/bringup2.log"
if ! grep -q "after:  (0, {'form': '0', 'name': ''})" "$OUT/bringup2.log"; then
    echo "FAIL  the bring-up did not release the motion service:"; grep -A3 "releasing" "$OUT/bringup2.log"; exit 1
fi
echo "PASS  g1-bringup.sh --sim released the motion service"
or_tail "$OUT/bringup2.log" python3 "$HERE/drive_sim_link.py" rebrought
