#!/usr/bin/env bash
# Run everything that can be verified without a robot, and say what each stage
# covers.
#
# Needs: numpy and pytest for the unit stage; ROS 2 sourced plus a built
# workspace for the other stages. Stages that cannot run are skipped with the
# reason. Exit: 0 every requested stage passed, 1 a stage failed, 2 a stage was
# skipped (--allow-skip: a skip passes). --docker needs only docker: it builds
# and runs every stage in tools/Dockerfile.test with the container network off.
# Every stage but unit fails on a sourced robot_bridge built from another clone,
# or older than the newest file under bridge/.
#
#   unit   pure Python: clients, adapters, goal order, observation, camera, tools/tests
#   cpp    gtests test_estop, test_gates, test_crc: e-stop latch, stop frames, remote keys,
#          target gates, the /lowcmd CRC against Unitree's SDK
#   wire   the real G1_bridge against a fake robot, body path
#   estop  the real G1_bridge's e-stop against a fake robot
#   gates  a fresh G1_bridge's target gates (range, speed, torque window, arming seed)
#          against a fake robot
#   hands  the real G1_bridge against fake Dex3 hands, three runs: plain, the left hand's
#          state stops (hands-stale), the left hand overheats (hands-hot)
#   config the real G1_bridge refuses broken copies of G1_config.yaml
#   sim    opt-in (--sim), host only: the real G1_bridge against g1_sim (MuJoCo): arm,
#          hold, /robot_cmd, remote damp
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
RUN_GATES=0
RUN_HANDS=0
RUN_CONFIG=0
RUN_SIM=0
RMW_CHOICE="auto"
STALE_AFTER="${STALE_AFTER:-3.5}"
HOT_AFTER="${HOT_AFTER:-3.5}"
PYTHON="${PYTHON:-python3}"
DOCKER=0
REBUILD=0
ALLOW_SKIP=0

usage() {
    cat <<'EOF'
Usage: tools/test.sh [options]

Run the no-hardware test stages. With no stage flag, runs all of them.
Exit 0: every requested stage passed; 1: a stage failed; 2: a stage was skipped (it did not run).

Options:
  --unit                pure Python tests only; needs no ROS
  --cpp                 gtests test_estop, test_gates and test_crc only; needs a built workspace
  --wire                body-path integration test only
  --estop               e-stop integration test only
  --gates               target-gate integration test only
  --hands               hand-path integration test only
  --config              broken-config refusal test only
  --sim                 G1_bridge against g1_sim only, not in the default set; needs g1_sim
                        built and $G1_MJCF, so not with --docker
  --rmw <impl>          cyclonedds | fastrtps | auto (default auto)
  --python <path>       interpreter for the unit stage (default python3, or $PYTHON)
  --stale-after <s>     hands-stale run: the left hand's state stops <s> seconds after
                        start_hand_control (default 3.5). 3.0 to 4.0: mid-stream
  --hot-after <s>       hands-hot run: the left hand reports over temperature at <s>
                        seconds, same clock and range (default 3.5)
  --docker              build and run the stages in a ROS 2 Humble container
                        (tools/Dockerfile.test), network off; workspace in $HB_TEST_WS
                        (default ~/.cache/humanoid_bridge/test_ws-<cksum of this clone's path>)
  --rebuild             with --docker: rebuild the image first
  --allow-skip          a skipped stage does not fail the run (exit 0, not 2)
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
        --gates) RUN_GATES=1 ;;
        --hands) RUN_HANDS=1 ;;
        --config) RUN_CONFIG=1 ;;
        --sim)   RUN_SIM=1 ;;
        --rmw)   RMW_CHOICE="${2:?--rmw needs cyclonedds, fastrtps or auto}"; shift ;;
        --python) PYTHON="${2:?--python needs an interpreter path}"; shift ;;
        --stale-after) STALE_AFTER="${2:?--stale-after needs seconds}"; shift ;;
        --hot-after) HOT_AFTER="${2:?--hot-after needs seconds}"; shift ;;
        --docker)  DOCKER=1 ;;
        --rebuild) REBUILD=1 ;;
        --allow-skip) ALLOW_SKIP=1 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
    shift
done

if [ "$RUN_UNIT$RUN_CPP$RUN_WIRE$RUN_ESTOP$RUN_GATES$RUN_HANDS$RUN_CONFIG$RUN_SIM" = "00000000" ]; then
    RUN_UNIT=1; RUN_CPP=1; RUN_WIRE=1; RUN_ESTOP=1; RUN_GATES=1; RUN_HANDS=1; RUN_CONFIG=1
fi

# 3.0 to 4.0 s after start_hand_control: earlier and the left hand never tracks,
# later and the driver has stopped.
for window in "--stale-after $STALE_AFTER" "--hot-after $HOT_AFTER"; do
    value="${window#* }"
    if ! [[ "$value" =~ ^[0-9]+(\.[0-9]+)?$ ]] \
       || ! awk -v s="$value" 'BEGIN { exit !(s >= 3.0 && s <= 4.0) }'; then
        echo "${window%% *} must be a number from 3.0 to 4.0 seconds; got '$value'" >&2
        exit 2
    fi
done

# --- docker ----------------------------------------------------------------
# Re-runs this script in the container on a workspace of its own, after an
# incremental build of this clone (mounted read-only). --network none: no DDS
# packet can leave the container, whatever the RMW settings.
run_in_docker() {
    local image="${HB_TEST_IMAGE:-humanoid-bridge-test:humble}"
    local key ws inner=()
    key="$(printf '%s' "$REPO" | cksum | cut -d' ' -f1)"
    ws="${HB_TEST_WS:-$HOME/.cache/humanoid_bridge/test_ws-$key}"
    command -v docker >/dev/null || { echo "--docker: no docker on PATH" >&2; exit 2; }
    if [ "$REBUILD" = 1 ] || ! docker image inspect "$image" >/dev/null 2>&1; then
        docker build -t "$image" -f "$HERE/Dockerfile.test" "$HERE"
    fi
    # Made here, as this user: docker would make a missing mount point as root.
    mkdir -p "$ws/src/humanoid_bridge"
    [ "$RUN_UNIT" = 1 ] && inner+=(--unit)
    [ "$RUN_CPP" = 1 ] && inner+=(--cpp)
    [ "$RUN_WIRE" = 1 ] && inner+=(--wire)
    [ "$RUN_ESTOP" = 1 ] && inner+=(--estop)
    [ "$RUN_GATES" = 1 ] && inner+=(--gates)
    [ "$RUN_HANDS" = 1 ] && inner+=(--hands)
    [ "$RUN_CONFIG" = 1 ] && inner+=(--config)
    inner+=(--stale-after "$STALE_AFTER" --hot-after "$HOT_AFTER")
    inner+=(--rmw "$RMW_CHOICE")
    [ "$ALLOW_SKIP" = 1 ] && inner+=(--allow-skip)
    echo "docker: $image, workspace $ws"
    # shellcheck disable=SC2016  # expanded in the container
    exec docker run --rm --init --network none \
        --user "$(id -u):$(id -g)" -e HOME=/tmp -e PYTEST_ADDOPTS="-p no:cacheprovider" \
        -v "$ws:/ws" -v "$REPO:/ws/src/humanoid_bridge:ro" \
        -w /ws/src/humanoid_bridge \
        "$image" bash -c '
            set -e
            . /opt/ros/humble/setup.bash
            tools/build.sh --workspace /ws -- --packages-up-to robot_bridge
            . /ws/install/setup.bash
            exec tools/test.sh "$@"' bash "${inner[@]}"
}

if [ "$DOCKER" = 1 ] && [ "$RUN_SIM" = 1 ]; then
    echo "--sim runs on the host: the test image has no MuJoCo and no G1 model" >&2
    exit 2
fi
if [ "$DOCKER" = 1 ]; then
    run_in_docker
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
SKIPPED=0

record() {
    RESULTS+=("$(printf '%-12s %-6s %s' "$1" "$2" "$3")")
    if [ "$2" = SKIP ]; then SKIPPED=$((SKIPPED + 1)); fi
}

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
    for mod in pytest numpy yaml zmq msgpack msgpack_numpy; do
        "$py" -c "import $mod" >/dev/null 2>&1 || missing="$missing $mod"
    done
    if [ -n "$missing" ]; then
        record unit SKIP "$py lacks$missing; tools/install-build-deps.sh (or --python <venv>/bin/python3)"
        return
    fi
    echo "=== unit ==="
    local out
    # No autoloaded plugin: a sourced ROS adds launch_testing's and ament's hooks, a venv its own.
    if out="$(cd "$REPO" && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 "$py" -m pytest -q 2>&1)"; then
        echo "$out" | tail -3
        record unit PASS "$(echo "$out" | tail -1)"
    else
        echo "$out" | tail -25
        record unit FAIL "$(echo "$out" | tail -1)"
        FAILED=1
    fi
}

# Why the sourced robot_bridge is not this clone's current build; nothing if it is.
# colcon rewrites build/<pkg>/colcon_build.rc on every build, a no-op one too.
stale_build() {
    local prefix ws build home newest
    prefix="$(ros2 pkg prefix robot_bridge)"
    ws="${prefix%/install/robot_bridge}"
    build="$ws/build/robot_bridge"
    if [ ! -f "$build/CMakeCache.txt" ] || [ ! -f "$build/colcon_build.rc" ]; then
        echo "no colcon build dir $build for the sourced $prefix; tools/build.sh, then source its install/setup.bash"
        return
    fi
    home="$(sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' "$build/CMakeCache.txt")"
    if [ "$(realpath -m "$home")" != "$(realpath -m "$REPO/bridge/robot_bridge")" ]; then
        echo "the sourced robot_bridge was built from $home, not this clone; tools/build.sh here, then source its install/setup.bash"
        return
    fi
    newest="$(find "$REPO/bridge" \( -name __pycache__ -o -name .pytest_cache \) -prune \
        -o -type f -newer "$build/colcon_build.rc" -print -quit)"
    if [ -n "$newest" ]; then
        echo "the sourced robot_bridge is older than ${newest#"$REPO/"}; tools/build.sh --workspace $ws, then source $ws/install/setup.bash"
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
    if [ -n "$BUILD_PROBLEM" ]; then
        record "$name" FAIL "$BUILD_PROBLEM"
        FAILED=1
        return
    fi
    echo "=== $name ==="
    if "$script" "$@"; then
        record "$name" PASS "$(basename "$script") $*"
    else
        record "$name" FAIL "$(basename "$script") $*"
        FAILED=1
    fi
}

stage_cpp() {
    local prefix bin name out
    if ! prefix="$(ros2 pkg prefix robot_bridge 2>/dev/null)"; then
        record cpp SKIP "robot_bridge not in the sourced workspace; tools/build.sh"
        return
    fi
    if [ -n "$BUILD_PROBLEM" ]; then
        record cpp FAIL "$BUILD_PROBLEM"
        FAILED=1
        return
    fi
    for name in test_estop test_gates test_crc; do
        bin="${prefix%/install/robot_bridge}/build/robot_bridge/$name"
        if [ ! -x "$bin" ]; then
            record cpp SKIP "no $bin; build with BUILD_TESTING on (colcon's default)"
            continue
        fi
        echo "=== cpp $name ==="
        if out="$("$bin" 2>&1)"; then
            echo "$out" | tail -1
            record cpp PASS "$name: $(echo "$out" | tail -1)"
        else
            # || true: no match (a crash) or head closing early must not end the run before the summary.
            echo "$out" | grep -E "FAILED|Failure|Expected|Which is|actual" | head -25 || true
            record cpp FAIL "$name: $(echo "$out" | tail -1)"
            FAILED=1
        fi
    done
}

BUILD_PROBLEM=""
if [ "$RUN_CPP$RUN_WIRE$RUN_ESTOP$RUN_GATES$RUN_HANDS$RUN_CONFIG" != 000000 ] && command -v ros2 >/dev/null \
   && ros2 pkg prefix robot_bridge >/dev/null 2>&1; then
    BUILD_PROBLEM="$(stale_build)"
fi

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
if [ "$RUN_GATES" = 1 ]; then
    stage_integration gates "$INTEGRATION/run_fake_gates_test.sh"
fi
if [ "$RUN_HANDS" = 1 ]; then
    stage_integration hands "$INTEGRATION/run_fake_hand_test.sh"
    stage_integration hands-stale "$INTEGRATION/run_fake_hand_test.sh" --stale-after "$STALE_AFTER"
    stage_integration hands-hot "$INTEGRATION/run_fake_hand_test.sh" --hot-after "$HOT_AFTER"
fi
if [ "$RUN_CONFIG" = 1 ]; then
    stage_integration config "$INTEGRATION/run_config_test.sh"
fi
if [ "$RUN_SIM" = 1 ]; then
    if command -v ros2 >/dev/null && ! ros2 pkg prefix g1_sim >/dev/null 2>&1; then
        record sim SKIP "g1_sim not in the sourced workspace; tools/build.sh"
    else
        stage_integration sim "$REPO/robot_stacks/g1/g1_sim/tests/integration/run_sim_bridge_test.sh"
    fi
fi

echo
echo "--- summary ---"
printf '%s\n' "${RESULTS[@]}"
if [ "$FAILED" = 1 ]; then
    exit 1
fi
if [ "$SKIPPED" -gt 0 ] && [ "$ALLOW_SKIP" = 0 ]; then
    echo "$SKIPPED requested stage(s) did not run: exit 2 (--allow-skip to pass anyway)"
    exit 2
fi
exit 0
