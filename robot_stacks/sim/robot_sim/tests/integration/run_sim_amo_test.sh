#!/usr/bin/env bash
# AMO stands the MuJoCo G1 through G1_bridge: robot_sim, G1_bridge, amo_controller, drive_amo_sim.py.
# Needs: the workspace with robot_bridge and unitree_hg sourced; AMO_PYTHON with CUDA torch,
# pinocchio and mujoco (default psi0's .venv-psi); PSI0_ROOT; the mjlab g1.xml ($G1_MJCF).
# Re-runs itself in a network namespace with only lo: no packet reaches a cabled robot.
set -euo pipefail

if [ -z "${AMO_SIM_NETNS:-}" ]; then
    exec unshare -rn env AMO_SIM_NETNS=1 bash "$0" "$@"
fi
ip link set lo up
ip link set lo multicast on
ip route add 224.0.0.0/4 dev lo

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../../../../.." && pwd)"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# shellcheck disable=SC1091
source "$REPO/bridge/robot_bridge/tests/integration/loopback_dds.sh"
PARAMS="$REPO/bridge/robot_bridge/params/G1_config.yaml"
AMO_PYTHON="${AMO_PYTHON:-${PSI0_ROOT:-$HOME/github/psi0}/.venv-psi/bin/python}"
OUT="${AMO_SIM_OUT:-$(mktemp -d)}"
export PYTHONPATH="$REPO/bridge/robot_bridge:$REPO/robot_stacks/g1/g1_stack:$REPO/robot_stacks/sim/robot_sim:$REPO/robot_stacks/g1/g1_camera:${PYTHONPATH:-}"

SIM_PID=""; BRIDGE_PID=""; AMO_PID=""
cleanup() {
    # Kill process groups: killing `ros2 run` alone orphans its child.
    for pid in "$AMO_PID" "$BRIDGE_PID" "$SIM_PID"; do
        [ -n "$pid" ] && kill -9 -- "-$pid" 2>/dev/null || true
    done
}
trap cleanup EXIT

command -v ros2 >/dev/null || { echo "source the ROS workspace first"; exit 1; }
[ -x "$AMO_PYTHON" ] || { echo "no AMO_PYTHON at $AMO_PYTHON"; exit 1; }
echo "logs: $OUT"

setsid "$AMO_PYTHON" -m robot_sim.node --robot g1 --no-viewer --no-camera > "$OUT/sim.log" 2>&1 &
SIM_PID=$!; disown "$SIM_PID"
sleep 4
grep -q "gantry on" "$OUT/sim.log" || { echo "sim did not come up:"; tail -20 "$OUT/sim.log"; exit 1; }

setsid ros2 run robot_bridge G1_bridge --ros-args -r __node:=robot_bridge \
    --params-file "$PARAMS" > "$OUT/bridge.log" 2>&1 &
BRIDGE_PID=$!; disown "$BRIDGE_PID"
sleep 4
grep -q "G1 type:" "$OUT/bridge.log" || { echo "bridge did not come up:"; tail -20 "$OUT/bridge.log"; exit 1; }

setsid "$AMO_PYTHON" -u -m g1_stack.nodes.amo_controller --tau-ff "${AMO_TAU_FF:-arms}" \
    > "$OUT/amo.log" 2>&1 &
AMO_PID=$!; disown "$AMO_PID"
for _ in $(seq 60); do grep -q "AMO -> /robot_cmd" "$OUT/amo.log" && break; sleep 0.5; done
grep -q "AMO -> /robot_cmd" "$OUT/amo.log" || { echo "amo_controller did not come up:"; tail -20 "$OUT/amo.log"; exit 1; }

if ! "$AMO_PYTHON" "$HERE/drive_amo_sim.py"; then
    echo; echo "amo log, last 20 lines:"; tail -20 "$OUT/amo.log"
    echo; echo "bridge log, last 30 lines:"; tail -30 "$OUT/bridge.log"
    echo; echo "sim log, last 10 lines:"; tail -10 "$OUT/sim.log"; exit 1
fi
clamps="$(grep -c "outside \[" "$OUT/bridge.log" || true)"
echo "note  bridge clamp lines: $clamps"
if grep -q "behind real time" "$OUT/sim.log"; then
    echo "WARN: the sim fell behind real time:"; grep "behind real time" "$OUT/sim.log" | tail -3
fi
