#!/usr/bin/env bash
# Runs the real G1_bridge's e-stop against a fake robot. No hardware. Exit 1 on failure.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$HERE/loopback_dds.sh"
PKG="$(cd "$HERE/../.." && pwd)"
PARAMS="$PKG/params/G1_config.yaml"
OUT="$(mktemp -d)"

export PYTHONPATH="$PKG:$HERE:${PYTHONPATH:-}"

FAKE_PID=""; BRIDGE_PID=""; DRIVER_PID=""; SECOND_PID=""
cleanup() {
    # Kill the process group: killing `ros2 run` alone orphans G1_bridge.
    [ -n "$BRIDGE_PID" ] && kill -9 -- "-$BRIDGE_PID" 2>/dev/null || true
    [ -n "$SECOND_PID" ] && kill -9 -- "-$SECOND_PID" 2>/dev/null || true
    [ -n "$FAKE_PID" ] && kill -9 "$FAKE_PID" 2>/dev/null || true
    [ -n "$DRIVER_PID" ] && kill -9 "$DRIVER_PID" 2>/dev/null || true
}
trap cleanup EXIT

command -v ros2 >/dev/null || { echo "source the ROS workspace first"; exit 1; }

python3 "$HERE/fake_g1.py" "$OUT/lowcmd.json" > "$OUT/fake.log" 2>&1 &
FAKE_PID=$!; disown "$FAKE_PID"  # no 'Killed' notice when cleanup kills it
sleep 2

# The driver holds a /lowcmd publisher before the bridge starts, and drops it once checked.
HOLDING="driver: holding /lowcmd"   # drive_estop.py prints it once the publisher exists
python3 "$HERE/drive_estop.py" "$PARAMS" "$OUT/bridge.log" > >(tee "$OUT/driver.log") 2>&1 &
DRIVER_PID=$!
for _ in $(seq 1 100); do
    grep -q "$HOLDING" "$OUT/driver.log" 2>/dev/null && break
    kill -0 "$DRIVER_PID" 2>/dev/null || break
    sleep 0.1
done
if ! grep -q "$HOLDING" "$OUT/driver.log" 2>/dev/null; then
    echo "driver did not print '$HOLDING' within 10 s"; exit 1
fi

setsid ros2 run robot_bridge G1_bridge --ros-args -r __node:=robot_bridge \
    --params-file "$PARAMS" > "$OUT/bridge.log" 2>&1 &
BRIDGE_PID=$!; disown "$BRIDGE_PID"  # no 'Killed' notice when cleanup kills it

if ! wait "$DRIVER_PID"; then
    echo; echo "bridge log, last 30 lines:"; tail -30 "$OUT/bridge.log"; exit 1
fi

# Ctrl-C in a startup wait: a second bridge waits on the first one's /lowcmd and must exit 0.
setsid ros2 run robot_bridge G1_bridge --ros-args -r __node:=robot_bridge \
    --params-file "$PARAMS" > "$OUT/second.log" 2>&1 &
SECOND_PID=$!
for _ in $(seq 1 100); do
    grep -q "publishers on /lowcmd" "$OUT/second.log" && break
    sleep 0.1
done
if ! grep -q "publishers on /lowcmd" "$OUT/second.log"; then
    echo "second bridge did not wait on /lowcmd within 10 s:"; tail -20 "$OUT/second.log"; exit 1
fi
kill -INT -- "-$SECOND_PID"
for _ in $(seq 1 50); do
    kill -0 "$SECOND_PID" 2>/dev/null || break
    sleep 0.1
done
if kill -0 "$SECOND_PID" 2>/dev/null; then
    printf '%-44s FAIL  still running 5 s after SIGINT\n' "SIGINT in the /lowcmd wait exits"; exit 1
fi
RC=0; wait "$SECOND_PID" || RC=$?
SECOND_PID=""
if [ "$RC" != 0 ]; then
    printf '%-44s FAIL  exit %s\n' "SIGINT in the /lowcmd wait exits" "$RC"; tail -5 "$OUT/second.log"; exit 1
fi
printf '%-44s PASS\n' "SIGINT in the /lowcmd wait exits"

# Last: SIGINT while armed, so the bridge does not survive it.
echo; echo "=== shutdown ==="
if ! python3 "$HERE/drive_shutdown.py" "$PARAMS" "$BRIDGE_PID" "$OUT/bridge.log"; then
    echo; echo "bridge log, last 30 lines:"; tail -30 "$OUT/bridge.log"; exit 1
fi
