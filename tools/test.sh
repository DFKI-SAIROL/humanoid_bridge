#!/usr/bin/env bash
# Run everything that can be verified without a robot, and say what each stage
# covers.
#
# Needs: numpy and pytest for the unit stage; ROS 2 sourced plus a built
# workspace for the two integration stages. Stages that cannot run are skipped
# with the reason, not failed.
#
#   unit   pure Python: clients, adapters, goal order, observation, camera
#   cpp    gtest test_estop: e-stop latch, stop frames, remote keys
#   wire   the real G1_bridge against a fake robot, body path
#   estop  the real G1_bridge's e-stop against a fake robot
#   console robot_console against the real G1_bridge and a fake robot
#   hands  the real G1_bridge against fake Dex3 hands, per-side release
#
# No stage touches hardware. Nothing here opens a real NIC: the DDS settings
# below pin discovery to loopback, overriding any inherited CYCLONEDDS_URI.
#
# Safe to re-run.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
INTEGRATION="$REPO/bridge/robot_bridge/tests/integration"

RUN_UNIT=0
RUN_CPP=0
RUN_WIRE=0
RUN_ESTOP=0
RUN_CONSOLE=0
RUN_HANDS=0
RMW_CHOICE="auto"
STALE_AFTER="${STALE_AFTER:-}"
PYTHON="${PYTHON:-python3}"

usage() {
    cat <<'EOF'
Usage: tools/test.sh [options]

Run the no-hardware test stages. With no stage flag, runs all of them.

Options:
  --unit                pure Python tests only; needs no ROS
  --cpp                 gtest test_estop only; needs a built workspace
  --wire                body-path integration test only
  --estop               e-stop integration test only
  --console             robot_console integration test only
  --hands               hand-path integration test only
  --rmw <impl>          cyclonedds | fastrtps | auto (default auto)
  --python <path>       interpreter for the unit stage (default python3, or $PYTHON)
  --stale-after <s>     hand stage: silence the left hand at <s> seconds to
                        exercise the per-side release. Must be 3.0 to 4.0
  -h, --help            this text

Discovery: CycloneDDS disables multicast on loopback, so a multi-process run on
one host finds nothing and the bridge looks like it never came up. When Cyclone
is the RMW this script sets CYCLONEDDS_URI for lo with multicast="true" itself,
replacing any inherited one (scripts/setup_unitree.sh points it at the robot).
Fast DDS gets ROS_LOCALHOST_ONLY=1.
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --unit)  RUN_UNIT=1 ;;
        --cpp)   RUN_CPP=1 ;;
        --wire)  RUN_WIRE=1 ;;
        --estop) RUN_ESTOP=1 ;;
        --console) RUN_CONSOLE=1 ;;
        --hands) RUN_HANDS=1 ;;
        --rmw)   RMW_CHOICE="${2:?--rmw needs cyclonedds, fastrtps or auto}"; shift ;;
        --python) PYTHON="${2:?--python needs an interpreter path}"; shift ;;
        --stale-after) STALE_AFTER="${2:?--stale-after needs seconds}"; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
    shift
done

if [ "$RUN_UNIT$RUN_CPP$RUN_WIRE$RUN_ESTOP$RUN_CONSOLE$RUN_HANDS" = "000000" ]; then
    RUN_UNIT=1; RUN_CPP=1; RUN_WIRE=1; RUN_ESTOP=1; RUN_CONSOLE=1; RUN_HANDS=1
fi

if [ -n "$STALE_AFTER" ] \
   && { [ "${STALE_AFTER%%.*}" -lt 3 ] || [ "${STALE_AFTER%%.*}" -ge 4 ]; }; then
    echo "--stale-after must be between 3.0 and 4.0 seconds; got $STALE_AFTER" >&2
    echo "    earlier and the left hand never tracks; later and the driver has stopped" >&2
    exit 2
fi

# --- DDS -------------------------------------------------------------------
LOOPBACK_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" multicast="true"/></Interfaces></General></Domain></CycloneDDS>'
case "$RMW_CHOICE" in
    cyclonedds)
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI="$LOOPBACK_URI" ;;
    fastrtps)
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=1
        unset CYCLONEDDS_URI ;;
    auto)
        # Never keep an inherited URI: after setup_unitree.sh it names the
        # robot's NIC, and the fake bridge would publish /lowcmd to the robot.
        if [ "${RMW_IMPLEMENTATION:-}" = "rmw_cyclonedds_cpp" ]; then
            export CYCLONEDDS_URI="$LOOPBACK_URI"
        else
            export ROS_LOCALHOST_ONLY=1
        fi ;;
    *) echo "--rmw must be cyclonedds, fastrtps or auto; got '$RMW_CHOICE'" >&2; exit 2 ;;
esac
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# --- stages ----------------------------------------------------------------
RESULTS=()
FAILED=0

record() { RESULTS+=("$(printf '%-8s %-6s %s' "$1" "$2" "$3")"); }

stage_unit() {
    local py
    py="$(command -v "$PYTHON" || true)"
    if [ -z "$py" ]; then
        record unit SKIP "no such interpreter: $PYTHON"
        return
    fi
    # Every module the unit tests import at collection time. cv2 is left out:
    # g1_camera imports it inside functions, on purpose.
    local missing=""
    for mod in pytest numpy zmq msgpack msgpack_numpy; do
        "$py" -c "import $mod" >/dev/null 2>&1 || missing="$missing $mod"
    done
    if [ -n "$missing" ]; then
        record unit SKIP "$py lacks$missing; tools/install-build-deps.sh (or --python <venv>/bin/python3)"
        return
    fi
    echo "=== unit ==="
    local out
    if out="$(cd "$REPO" && "$py" -m pytest -q 2>&1)"; then
        echo "$out" | tail -3
        record unit PASS "$(echo "$out" | tail -1)"
    else
        echo "$out" | tail -25
        record unit FAIL "$(echo "$out" | tail -1)"
        FAILED=1
    fi
}

stage_integration() {   # $1 name, $2 script, rest: script args
    local name="$1" script="$2"; shift 2
    if ! command -v ros2 >/dev/null; then
        record "$name" SKIP "no ros2 on PATH; source the workspace (tools/build.sh)"
        return
    fi
    if ! ros2 pkg prefix robot_bridge >/dev/null 2>&1; then
        record "$name" SKIP "robot_bridge not in the sourced workspace; tools/build.sh"
        return
    fi
    echo "=== $name ==="
    if "$script" "$@"; then
        record "$name" PASS "$(basename "$script")"
    else
        record "$name" FAIL "$(basename "$script")"
        FAILED=1
    fi
}

stage_cpp() {
    local prefix bin
    if ! prefix="$(ros2 pkg prefix robot_bridge 2>/dev/null)"; then
        record cpp SKIP "robot_bridge not in the sourced workspace; tools/build.sh"
        return
    fi
    bin="${prefix%/install/robot_bridge}/build/robot_bridge/test_estop"
    if [ ! -x "$bin" ]; then
        record cpp SKIP "no $bin; build with BUILD_TESTING on (colcon's default)"
        return
    fi
    echo "=== cpp ==="
    local out
    if out="$("$bin" 2>&1)"; then
        echo "$out" | tail -1
        record cpp PASS "$(echo "$out" | tail -1)"
    else
        echo "$out" | grep -E "FAILED|Failure|Expected|Which is|actual" | head -25
        record cpp FAIL "$(echo "$out" | tail -1)"
        FAILED=1
    fi
}

if [ "$RUN_UNIT" = 1 ]; then
    stage_unit
fi
if [ "$RUN_CPP" = 1 ]; then
    if command -v ros2 >/dev/null; then stage_cpp; else record cpp SKIP "no ros2 on PATH"; fi
fi
if [ "$RUN_WIRE" = 1 ]; then
    stage_integration wire "$INTEGRATION/run_fake_wire_test.sh"
fi
if [ "$RUN_ESTOP" = 1 ]; then
    stage_integration estop "$INTEGRATION/run_fake_estop_test.sh"
fi
if [ "$RUN_CONSOLE" = 1 ]; then
    stage_integration console "$INTEGRATION/run_fake_console_test.sh"
fi
if [ "$RUN_HANDS" = 1 ]; then
    # run_fake_hand_test.sh reads STALE_AFTER from the environment.
    if [ -n "$STALE_AFTER" ]; then export STALE_AFTER; else unset STALE_AFTER; fi
    stage_integration hands "$INTEGRATION/run_fake_hand_test.sh"
fi

echo
echo "--- summary ---"
printf '%s\n' "${RESULTS[@]}"
exit "$FAILED"
