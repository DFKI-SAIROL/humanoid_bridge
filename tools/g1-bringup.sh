#!/usr/bin/env bash
# Bring the G1 bridge up in the order that works, and optionally arm it.
#
#   release /lowcmd  ->  start G1_bridge  ->  wait for it  ->  arm
#
# Needs: a sourced ROS workspace (scripts/setup_unitree.sh), and for the
# release step, unitree_sdk2py on the interpreter that runs it.
#
# ARMING MOVES THE ROBOT. start_control ramps every one of the 29 joints, legs
# included, to the pose you pass, over `duration` (2 s). It is off unless you
# ask for it twice: an --arm-* flag and --robot-is-clear.
#
# The bridge does not check the length of default_position: a list that is not
# exactly 29 long is accepted, answers success, and moves the robot to ready_q_
# instead (bridge_core.cpp:549, G1_bridge.cpp:322). This script checks it.
#
# Safe to re-run: it starts nothing that is already running, and re-arming an
# armed bridge is a fresh ramp, not an error.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"

PARAMS="$REPO/bridge/robot_bridge/params/G1_config.yaml"
IFACE="${BRIDGE_IFACE:-}"
ARM_MODE=""          # "", "measured", or "explicit"
ARM_VALUES=""
ROBOT_IS_CLEAR=0
DO_RELEASE=1
DRY_RUN=0
WAIT_SECONDS=20
NUM_JOINT=29

usage() {
    cat <<'EOF'
Usage: tools/g1-bringup.sh [options]

Release /lowcmd, start G1_bridge, wait for it. Arms only if told to twice.

Options:
  --iface <nic>          DDS interface for the release step (default $BRIDGE_IFACE)
  --params <path>        bridge params file (default bridge/robot_bridge/params/G1_config.yaml)
  --no-release           skip the /lowcmd release (already released, or a fake robot)
  --arm-from-measured    arm to the pose the robot currently holds
  --arm-from "<29 nums>" arm to an explicit pose, length-checked here
  --robot-is-clear       required for any --arm-*. Gantry, e-stop, safety gate first
  --wait <s>             seconds to wait for the bridge (default 20)
  --dry-run              print every command, run none
  -h, --help             this text

Stop:  ros2 service call /stop_control std_srvs/srv/Trigger {}
Neither stop nor an abort releases the joints: the robot holds its last
commanded pose at full kp/kd.
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --iface)   IFACE="${2:?--iface needs a NIC}"; shift ;;
        --params)  PARAMS="${2:?--params needs a path}"; shift ;;
        --no-release) DO_RELEASE=0 ;;
        --arm-from-measured) ARM_MODE="measured" ;;
        --arm-from) ARM_MODE="explicit"; ARM_VALUES="${2:?--arm-from needs 29 numbers}"; shift ;;
        --robot-is-clear) ROBOT_IS_CLEAR=1 ;;
        --wait)    WAIT_SECONDS="${2:?--wait needs seconds}"; shift ;;
        --dry-run) DRY_RUN=1 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
    shift
done

say()  { printf '== %s\n' "$*"; }
run()  { if [ "$DRY_RUN" = 1 ]; then printf '   '; printf '%q ' "$@"; printf '\n'; else "$@"; fi; }

# --- argument checks -------------------------------------------------------
# Before the environment checks: a safety refusal must not depend on ROS.
if [ -n "$ARM_MODE" ] && [ "$ROBOT_IS_CLEAR" != 1 ]; then
    echo "refusing to arm without --robot-is-clear." >&2
    echo "start_control ramps all $NUM_JOINT joints, legs included, over 2 s." >&2
    echo "Gantry, e-stop and the safety gate first, then add --robot-is-clear." >&2
    exit 2
fi

if [ "$ARM_MODE" = "explicit" ]; then
    # shellcheck disable=SC2086
    set -- $ARM_VALUES
    if [ "$#" -ne "$NUM_JOINT" ]; then
        echo "--arm-from needs exactly $NUM_JOINT numbers, got $#." >&2
        echo "The bridge would accept this, answer success, and move to ready_q_ instead." >&2
        exit 2
    fi
    for v in "$@"; do
        case "$v" in
            ''|*[!0-9eE.+-]*) echo "--arm-from: '$v' is not a number" >&2; exit 2 ;;
        esac
    done
    ARM_VALUES="$*"
fi

command -v ros2 >/dev/null || {
    echo "ros2 not on PATH. Source the workspace first:" >&2
    echo "    BRIDGE_IFACE=<nic> source scripts/setup_unitree.sh" >&2
    exit 1; }
[ -f "$PARAMS" ] || { echo "no params file at $PARAMS" >&2; exit 1; }

# --- 1. release /lowcmd ----------------------------------------------------
if [ "$DO_RELEASE" = 1 ]; then
    say "releasing /lowcmd"
    if [ -z "$IFACE" ]; then
        echo "no interface for the release step: pass --iface, set BRIDGE_IFACE," >&2
        echo "or pass --no-release if nothing else holds /lowcmd." >&2
        exit 2
    fi
    run "$HERE/g1-release-lowcmd.py" --iface "$IFACE"
else
    say "skipping the /lowcmd release (--no-release)"
fi

# --- 2. start the bridge ---------------------------------------------------
say "starting G1_bridge"
BRIDGE_LOG="$(mktemp -t g1-bringup-XXXXXX.log)"
if [ "$DRY_RUN" = 1 ]; then
    printf '   '; printf '%q ' ros2 run robot_bridge G1_bridge --ros-args \
        --params-file "$PARAMS"; printf '\n'
else
    # setsid: `ros2 run` is a launcher, so killing it orphans G1_bridge, which
    # keeps holding /lowcmd and blocks the next run.
    setsid ros2 run robot_bridge G1_bridge --ros-args --params-file "$PARAMS" \
        > "$BRIDGE_LOG" 2>&1 &
    BRIDGE_PID=$!
    echo "   pid $BRIDGE_PID, log $BRIDGE_LOG"
    echo "   stop it with: kill -- -$BRIDGE_PID"
fi

# --- 3. wait for it --------------------------------------------------------
say "waiting for the bridge (up to ${WAIT_SECONDS}s)"
if [ "$DRY_RUN" = 1 ]; then
    echo "   grep 'G1 type:' in the bridge log"
else
    ready=0
    for _ in $(seq 1 "$((WAIT_SECONDS * 2))"); do
        if grep -q "G1 type:" "$BRIDGE_LOG" 2>/dev/null; then ready=1; break; fi
        if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then break; fi
        sleep 0.5
    done
    if [ "$ready" != 1 ]; then
        echo "the bridge did not come up. Last lines:" >&2
        tail -20 "$BRIDGE_LOG" >&2
        echo >&2
        echo "'Waiting for publisher on /lowstate': the robot is not publishing, or" >&2
        echo "CycloneDDS is on the wrong NIC. Set BRIDGE_IFACE and re-source." >&2
        echo "'Detected N publishers on /lowcmd': something still holds the topic." >&2
        echo "Run without --no-release, or press L2 + R2 on the controller." >&2
        exit 1
    fi
    echo "   up"
    echo "   e-stop: L1+R1 damp, R1+R2 hold (latched); clear with: ros2 service call /estop_reset std_srvs/srv/Trigger"
fi

# --- 4. arm ----------------------------------------------------------------
if [ -z "$ARM_MODE" ]; then
    say "not arming. The bridge drops every /robot_cmd until start_control is called"
    echo "   arm with: $0 --arm-from-measured --robot-is-clear"
    exit 0
fi

if [ "$ARM_MODE" = "measured" ]; then
    say "reading the current pose"
    if [ "$DRY_RUN" = 1 ]; then
        printf '   '; printf '%q ' "$HERE/g1-measured-pose.py"; printf '\n'
        ARM_VALUES="<29 measured values>"
    else
        ARM_VALUES="$("$HERE/g1-measured-pose.py" --num-joint "$NUM_JOINT")"
        echo "   $ARM_VALUES"
    fi
fi

say "ARMING: this moves the robot over 2 s"
ARM_REQUEST="{default_position: [$(echo "$ARM_VALUES" | tr ' ' ',')]}"
if [ "$DRY_RUN" = 1 ]; then
    # Printed plainly, not %q-escaped: this one is meant to be copy-pasted.
    echo "   ros2 service call /start_control \\"
    echo "       bridge_interface/srv/SetDefaultPosition '$ARM_REQUEST'"
else
    ros2 service call /start_control bridge_interface/srv/SetDefaultPosition "$ARM_REQUEST"
fi

echo
echo "start_control answers success=false only while an e-stop is latched, and"
echo "success=true for a wrong length, so check the ramp in /lowcmd rather than the"
echo "reply. The first /robot_cmd cuts the ramp short, so arm before a client streams."
