#!/usr/bin/env bash
# Runs the real G1_bridge's hand path against fake Dex3 hands. No hardware.
# Exit 1 on failure.
#
# The bridge blocks in its constructor until something publishes /lowstate, so
# fake_g1 runs too. That doubles as the check that the body path is untouched.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$HERE/loopback_dds.sh"
PKG="$(cd "$HERE/../.." && pwd)"
PARAMS="$PKG/params/G1_config.yaml"
OUT="$(mktemp -d)"

# STALE_AFTER=<seconds> silences the left hand mid-stream, to exercise the
# per-side freshness release. The clock starts at the first /dex3/left/cmd,
# which is start_hand_control, so it must land after the 2.5 s ramp-and-hold
# and before the driver stops about 2.4 s later, or the left hand never tracks
# and the run is not testing what it looks like it is testing. 3.5 is the
# middle of that window.
STALE="${STALE_AFTER:-}"
if [ -n "$STALE" ] && { [ "${STALE%%.*}" -lt 3 ] || [ "${STALE%%.*}" -ge 4 ]; }; then
    echo "STALE_AFTER must be between 3.0 and 4.0 seconds; got $STALE"; exit 1
fi

export PYTHONPATH="$PKG:$HERE:${PYTHONPATH:-}"

FAKE_PID=""; HAND_PID=""; BRIDGE_PID=""
cleanup() {
    # ros2 run is a launcher: killing it orphans G1_bridge, which keeps holding
    # /lowcmd and blocks the next run. Kill the whole process group.
    [ -n "$BRIDGE_PID" ] && kill -9 -- "-$BRIDGE_PID" 2>/dev/null || true
    [ -n "$HAND_PID" ] && kill -9 "$HAND_PID" 2>/dev/null || true
    [ -n "$FAKE_PID" ] && kill -9 "$FAKE_PID" 2>/dev/null || true
}
trap cleanup EXIT

command -v ros2 >/dev/null || { echo "source the ROS workspace first"; exit 1; }

# The fakes first: /lowstate and /dex3/*/state must exist, and nothing may
# publish /lowcmd or /dex3/*/cmd.
python3 "$HERE/fake_g1.py" "$OUT/lowcmd.json" > "$OUT/fake_g1.log" 2>&1 &
FAKE_PID=$!
python3 "$HERE/fake_dex3.py" "$OUT/handcmd.json" ${STALE:+--stale-after "$STALE"} \
    > "$OUT/fake_dex3.log" 2>&1 &
HAND_PID=$!
sleep 2

setsid ros2 run robot_bridge G1_bridge --ros-args -r __node:=robot_bridge \
    --params-file "$PARAMS" > "$OUT/bridge.log" 2>&1 &
BRIDGE_PID=$!
sleep 4

if ! grep -q "G1 type:" "$OUT/bridge.log"; then
    echo "bridge did not come up:"; tail -20 "$OUT/bridge.log"; exit 1
fi
if ! grep -q "Dex3 hand path ready" "$OUT/bridge.log"; then
    echo "hand path did not load; check the hand_* parameters:"
    grep -i hand "$OUT/bridge.log" | tail -20; exit 1
fi

python3 "$HERE/drive_hand_cmd.py" --send-over-limit --send-nan

sleep 1
kill -TERM "$HAND_PID"; wait "$HAND_PID" 2>/dev/null || true; HAND_PID=""
kill -TERM "$FAKE_PID"; wait "$FAKE_PID" 2>/dev/null || true; FAKE_PID=""

echo
python3 "$HERE/check_hand_cmd.py" "$OUT/handcmd.json" \
    --expect-clamped --expect-nan-dropped \
    ${STALE:+--expect-left-released}

echo
# Non-interference: this run never called start_control, so the hand path must
# not have started body control. Wire handStarted_ to controlStarted_ and this
# is what catches it. The body path's own checks live in run_fake_wire_test.sh.
python3 - "$OUT/lowcmd.json" <<'EOF'
import json, sys

with open(sys.argv[1]) as handle:
    samples = json.load(handle)

ok = not samples
print("{:<34} {}  {}".format(
    "hand path left the body idle", "PASS" if ok else "FAIL",
    "{} /lowcmd samples, wanted 0".format(len(samples))))
sys.exit(0 if ok else 1)
EOF
