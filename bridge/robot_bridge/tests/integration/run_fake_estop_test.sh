#!/usr/bin/env bash
# Runs the real G1_bridge's e-stop against a fake robot. No hardware. Exit 1 on failure.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$HERE/harness.sh"

FAKE_PID=""; BRIDGE_PID=""; DRIVER_PID=""; SECOND_PID=""
cleanup() {
    kill_group "$BRIDGE_PID"
    kill_group "$SECOND_PID"
    kill_pid "$FAKE_PID"
    kill_pid "$DRIVER_PID"
}
trap cleanup EXIT

require_ros

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

start_bridge "$OUT/bridge.log"
BRIDGE_PID=$!; disown "$BRIDGE_PID"

or_tail "$OUT/bridge.log" wait "$DRIVER_PID"

# Ctrl-C in a startup wait: a second bridge waits on the first one's /lowcmd and must exit 0.
start_bridge "$OUT/second.log"
SECOND_PID=$!  # not disowned: waited for below
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
SIGINT_CHECK="SIGINT in the /lowcmd wait exits"
if kill -0 "$SECOND_PID" 2>/dev/null; then
    verdict "$SIGINT_CHECK" FAIL "still running 5 s after SIGINT"; exit 1
fi
RC=0; wait "$SECOND_PID" || RC=$?
SECOND_PID=""
if [ "$RC" != 0 ]; then
    verdict "$SIGINT_CHECK" FAIL "exit $RC"; tail -5 "$OUT/second.log"; exit 1
fi
verdict "$SIGINT_CHECK" PASS

# Last: SIGINT while armed, so the bridge does not survive it.
echo; echo "=== shutdown ==="
or_tail "$OUT/bridge.log" python3 "$HERE/drive_shutdown.py" "$PARAMS" "$BRIDGE_PID" "$OUT/bridge.log"
