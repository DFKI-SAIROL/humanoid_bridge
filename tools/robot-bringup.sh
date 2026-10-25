#!/usr/bin/env bash
# Bring a robot's bridge up in the order that works, and optionally arm it. --robot g1 (default) | t1.
#
#   G1: release /lowcmd  ->  start G1_bridge  ->  wait for it  ->  arm
#   T1: start T1_bridge  ->  wait for it  ->  arm (the bridge asks for custom mode on arming)
#
# Sources ROS and the workspace's scripts/setup_unitree.sh (T1: setup_booster.sh) itself. The G1's
# release step needs unitree_sdk2py: $UNITREE_PYTHON, default the lab venv. tools/g1-bringup.sh is
# this script with --robot g1.
#
# ARMING MOVES THE ROBOT. start_control ramps every joint (G1 29, T1 23), legs
# included, to the pose you pass, over `duration` (2 s). It is off unless you
# ask for it twice: an --arm-* flag and --robot-is-clear.
#
# The bridge refuses a default_position of another length (empty = ready_q_);
# this script checks the length first anyway. Arming also needs the remote on.
#
# Refuses while a robot_sim node is visible on the robot NIC (simulated /lowstate).
#
# --sim: against robot_sim on loopback DDS (robot_stacks/sim/robot_sim/README.md); still
# releases, from robot_sim's fake motion switcher.
#
# Safe to re-run: with one G1_bridge already up it only arms; with two or more
# it refuses (each writes /lowcmd). Only bridges in this network namespace
# count: a test's bridge in its own namespace cannot reach the robot.
#
# --foreground (for a supervisor such as robot_web_commander): same checks and
# release, then execs the bridge in this process; never arms.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"

ROBOT=g1
PARAMS=""
IFACE="${BRIDGE_IFACE:-enp4s0}"
IFACE_GIVEN=0
WS="${BRIDGE_WS:-$HOME/bridge_ws_estop}"
LAB_REPO="${LAB_REPO:-$HOME/github/humanoid-locoman-vla}"
UNITREE_PYTHON="${UNITREE_PYTHON:-}"
ARM_MODE=""          # "", "measured", or "explicit"
ARM_VALUES=""
ROBOT_IS_CLEAR=0
DO_RELEASE=1
DO_STOP=0
FOREGROUND=0
SIM=0
DRY_RUN=0
WAIT_SECONDS=20

usage() {
    cat <<'EOF'
Usage: tools/robot-bringup.sh [--robot g1|t1] [options]

G1: release /lowcmd, start G1_bridge, wait for it. T1: start T1_bridge, wait for it.
Arms only if told to twice.

Options:
  --robot <g1|t1>        which robot (default g1)
  --iface <nic>          DDS interface (default $BRIDGE_IFACE or enp4s0)
  --workspace <ws>       colcon workspace that builds this clone (default $BRIDGE_WS or ~/bridge_ws_estop)
  --params <path>        bridge params file (default bridge/robot_bridge/params/<G1|T1>_config.yaml)
  --no-release           G1: skip the /lowcmd release (already released, or a fake robot)
  --arm-from-measured    arm to the pose the robot currently holds
  --arm-from "<N nums>"  arm to an explicit pose (G1 29, T1 23), length-checked here
  --robot-is-clear       required for any --arm-*. Gantry, e-stop, safety gate first
  --stop                 stop every robot bridge (SIGINT, then SIGKILL), start nothing
  --foreground           release, then exec the bridge in this process (log on stdout);
                         refuses any --arm-* and a running bridge. Stop: SIGINT
                         to the process group
  --sim                  robot_sim instead of the robot: DDS on loopback only (--iface lo),
                         no robot_sim refusal; the G1 still releases /lowcmd
  --wait <s>             seconds to wait for the bridge (default 20)
  --dry-run              print every command, run none
  -h, --help             this text

Environment: UNITREE_PYTHON (interpreter with unitree_sdk2py; default
$LAB_REPO/.venv38/bin/python if present, else python3), LAB_REPO
(~/github/humanoid-locoman-vla).

Stop the bridge:  tools/robot-bringup.sh --stop
An armed bridge sends damp frames for 1 s, then /lowcmd goes silent, and the
motors go passive about 0.4 s later (docs/estop.md, G0): a standing robot
falls. Gantry first.
/stop_control keeps the bridge up and holds the measured pose.
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --robot)   ROBOT="${2:?--robot needs g1 or t1}"; shift ;;
        --iface)   IFACE="${2:?--iface needs a NIC}"; IFACE_GIVEN=1; shift ;;
        --workspace) WS="${2:?--workspace needs a path}"; shift ;;
        --params)  PARAMS="${2:?--params needs a path}"; shift ;;
        --no-release) DO_RELEASE=0 ;;
        --arm-from-measured) ARM_MODE="measured" ;;
        --arm-from) ARM_MODE="explicit"; ARM_VALUES="${2:?--arm-from needs the joint values}"; shift ;;
        --robot-is-clear) ROBOT_IS_CLEAR=1 ;;
        --stop)    DO_STOP=1 ;;
        --foreground) FOREGROUND=1 ;;
        --sim)     SIM=1 ;;
        --wait)    WAIT_SECONDS="${2:?--wait needs seconds}"; shift ;;
        --dry-run) DRY_RUN=1 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
    shift
done

case "$ROBOT" in
    g1) BIN=G1_bridge; NUM_JOINT=29; READY="G1 type:"; SETUP=setup_unitree.sh
        ESTOP_HINT="R1+R2 damp, L1+R1 hold" ;;
    t1) BIN=T1_bridge; NUM_JOINT=23; READY="T1 type:"; SETUP=setup_booster.sh; DO_RELEASE=0
        ESTOP_HINT="RB+RT damp, LB+RB hold (the gamepad)" ;;
    *) echo "--robot must be g1 or t1, not '$ROBOT'" >&2; exit 2 ;;
esac
[ -n "$PARAMS" ] || PARAMS="$REPO/bridge/robot_bridge/params/${ROBOT^^}_config.yaml"
BRIDGES=(G1_bridge T1_bridge)   # one robot per host: any of them counts

say()  { printf '== %s\n' "$*"; }
BRIDGE_LOG=""
log_hint() {   # the bridge runs detached: its output is only in the log
    if [ -n "$BRIDGE_LOG" ] && [ "$DRY_RUN" != 1 ]; then echo "   bridge log: tail -f $BRIDGE_LOG"; fi
}
run()  { if [ "$DRY_RUN" = 1 ]; then printf '   '; printf '%q ' "$@"; printf '\n'; else "$@"; fi; }

MY_NETNS="$(readlink /proc/self/ns/net)"
robot_bridges() {   # robot bridge pids in this network namespace
    local p
    for p in $(for b in "${BRIDGES[@]}"; do pgrep -x "$b" || true; done); do
        if [ "$(readlink "/proc/$p/ns/net" 2>/dev/null)" = "$MY_NETNS" ]; then echo "$p"; fi
    done
}
other_bridges() {   # bridge pids in other namespaces (tests); they cannot reach the robot
    local p
    for p in $(for b in "${BRIDGES[@]}"; do pgrep -x "$b" || true; done); do
        if [ "$(readlink "/proc/$p/ns/net" 2>/dev/null)" != "$MY_NETNS" ]; then echo "$p"; fi
    done
}
note_others() {
    local others
    others="$(other_bridges | tr '\n' ' ')"
    if [ -n "$others" ]; then echo "   note: a bridge in another network namespace, ignored: $others"; fi
}

# --- stop --------------------------------------------------------------------
if [ "$DO_STOP" = 1 ]; then
    mapfile -t pids < <(robot_bridges)
    note_others
    if [ "${#pids[@]}" -eq 0 ]; then say "no robot bridge runs"; exit 0; fi
    say "stopping bridge ${pids[*]}: damp 1 s, then its command topic goes silent (G1: motors passive ~0.4 s later)"
    for p in "${pids[@]}"; do
        # The process group: `ros2 run` is a launcher, and killing it alone orphans the bridge.
        run kill -INT -- "-$(ps -o pgid= -p "$p" | tr -d ' ')" || true
    done
    [ "$DRY_RUN" = 1 ] && exit 0
    for _ in $(seq 1 10); do
        [ -z "$(robot_bridges)" ] && break
        sleep 0.5
    done
    # A bridge can log "Bridge stopped." and still not exit: SIGKILL it.
    mapfile -t left < <(robot_bridges)
    if [ "${#left[@]}" -gt 0 ]; then
        echo "   still running 5 s after SIGINT: ${left[*]} -> SIGKILL"
        kill -9 "${left[@]}" 2>/dev/null || true
        sleep 0.5
    fi
    if [ -n "$(robot_bridges)" ]; then echo "could not stop: $(robot_bridges | tr '\n' ' ')" >&2; exit 1; fi
    echo "   stopped"
    exit 0
fi

# --- argument checks -------------------------------------------------------
# Before the environment checks: a safety refusal must not depend on ROS.
if [ "$SIM" = 1 ]; then
    if [ "$IFACE_GIVEN" = 1 ] && [ "$IFACE" != lo ]; then
        echo "refusing --sim with --iface $IFACE: the sim runs on lo only." >&2
        exit 2
    fi
    IFACE=lo
fi
if [ "$FOREGROUND" = 1 ] && [ -n "$ARM_MODE" ]; then
    echo "refusing --arm-* with --foreground: the bridge never returns to arm it." >&2
    echo "Arm from the page, or with: ros2 service call /start_control ..." >&2
    exit 2
fi
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
        echo "The bridge refuses it too; an empty list arms to ready_q_." >&2
        exit 2
    fi
    for v in "$@"; do
        case "$v" in
            ''|*[!0-9eE.+-]*) echo "--arm-from: '$v' is not a number" >&2; exit 2 ;;
        esac
    done
    ARM_VALUES="$*"
fi

[ -f "$PARAMS" ] || { echo "no params file at $PARAMS" >&2; exit 1; }

# --- environment: ROS, CycloneDDS on $IFACE, the workspace -----------------
WS_CLONE="$WS/src/humanoid_bridge"
if [ "$(cd "$WS_CLONE" 2>/dev/null && pwd -P)" != "$(cd "$REPO" && pwd -P)" ]; then
    echo "$WS_CLONE is not this clone ($REPO)." >&2
    echo "Run the script from the clone the workspace builds, or pass --workspace." >&2
    exit 1
fi
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [ "$SIM" = 1 ]; then
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    # shellcheck disable=SC1091
    source "$WS/install/setup.bash"
    # shellcheck disable=SC1091
    source "$WS_CLONE/bridge/robot_bridge/tests/integration/loopback_dds.sh"
else
    export BRIDGE_IFACE="$IFACE"
    # shellcheck disable=SC1090  # setup_unitree.sh or setup_booster.sh
    source "$WS_CLONE/scripts/$SETUP" >/dev/null
fi
set -u
say "workspace $WS, CycloneDDS on $IFACE"

# --- a sim on the robot network? -------------------------------------------
# Read-only, so also in --dry-run. A robot_sim here feeds the bridge simulated state.
if [ "$SIM" = 1 ]; then
    say "--sim: robot_sim expected on lo, no check"
else
    say "looking for a robot_sim node on $IFACE"
    if timeout 15 ros2 node list --no-daemon --spin-time 2 2>/dev/null | grep -qx /robot_sim; then
        echo "refused: a robot_sim node is visible on $IFACE; $BIN would read simulated state." >&2
        echo "Stop it. A sim runs with loopback DDS only (robot_stacks/sim/robot_sim/README.md)." >&2
        exit 1
    fi
    echo "   none"
fi

# --- 0. already running? ---------------------------------------------------
mapfile -t running < <(robot_bridges)
note_others
if [ "${#running[@]}" -gt 1 ]; then
    echo "refused: ${#running[@]} robot bridges run (${running[*]}); each writes its robot's commands." >&2
    echo "Stop them: $0 --stop" >&2
    exit 1
fi
if [ "${#running[@]}" -eq 1 ] && [ "$FOREGROUND" = 1 ]; then
    echo "refused: a bridge already runs (pid ${running[0]}); --foreground would start a second." >&2
    echo "Stop it: $0 --stop" >&2
    exit 1
fi
if [ "${#running[@]}" -eq 1 ]; then
    say "a bridge already runs (pid ${running[0]}): no release, no second bridge"
    log="$(readlink "/proc/${running[0]}/fd/1" 2>/dev/null || true)"
    if [ -f "$log" ]; then BRIDGE_LOG="$log"; fi
    echo "   restart it: $0 --stop, then run this again"
else
    # --- 1. release /lowcmd ------------------------------------------------
    if [ "$DO_RELEASE" = 1 ]; then
        if [ -z "$UNITREE_PYTHON" ]; then
            UNITREE_PYTHON=python3
            [ -x "$LAB_REPO/.venv38/bin/python" ] && UNITREE_PYTHON="$LAB_REPO/.venv38/bin/python"
        fi
        say "releasing /lowcmd ($UNITREE_PYTHON)"
        # env -u PYTHONPATH: ROS's Python 3.10 paths break another interpreter's imports.
        run env -u PYTHONPATH "$UNITREE_PYTHON" "$HERE/g1-release-lowcmd.py" --iface "$IFACE"
    elif [ "$ROBOT" = t1 ]; then
        say "no release on the T1: T1_bridge asks for custom mode when it arms"
    else
        say "skipping the /lowcmd release (--no-release)"
    fi

    # --- 2. start the bridge -----------------------------------------------
    if [ "$FOREGROUND" = 1 ]; then
        say "exec $BIN in the foreground; stop: SIGINT to this process group"
        # exec keeps the supervisor's pid and group; its signals reach `ros2 run` and the bridge.
        run exec ros2 run robot_bridge "$BIN" --ros-args --params-file "$PARAMS"
        exit 0   # --dry-run only
    fi
    say "starting $BIN"
    BRIDGE_LOG="$(mktemp -t robot-bringup-XXXXXX.log)"
    if [ "$DRY_RUN" = 1 ]; then
        printf '   '; printf '%q ' ros2 run robot_bridge "$BIN" --ros-args \
            --params-file "$PARAMS"; printf '\n'
    else
        # setsid: `ros2 run` is a launcher, so killing it orphans the bridge, which
        # keeps holding the command topic and blocks the next run.
        setsid ros2 run robot_bridge "$BIN" --ros-args --params-file "$PARAMS" \
            > "$BRIDGE_LOG" 2>&1 &
        BRIDGE_PID=$!
        echo "   pid $BRIDGE_PID, log $BRIDGE_LOG"
        echo "   stop it with: $0 --stop"
    fi

    # --- 3. wait for it ----------------------------------------------------
    say "waiting for the bridge (up to ${WAIT_SECONDS}s)"
    if [ "$DRY_RUN" = 1 ]; then
        echo "   grep '$READY' in the bridge log"
    else
        ready=0
        for _ in $(seq 1 "$((WAIT_SECONDS * 2))"); do
            if grep -q "$READY" "$BRIDGE_LOG" 2>/dev/null; then ready=1; break; fi
            if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then break; fi
            sleep 0.5
        done
        if [ "$ready" != 1 ]; then
            echo "the bridge did not come up. Last lines:" >&2
            tail -20 "$BRIDGE_LOG" >&2
            echo >&2
            echo "'Waiting for publisher on /lowstate': the robot is not publishing, or" >&2
            echo "CycloneDDS is on the wrong NIC: pass --iface." >&2
            echo "'Detected N publishers on /lowcmd': something still holds the topic." >&2
            echo "Run without --no-release, or press L2 + R2 on the controller." >&2
            exit 1
        fi
        echo "   up"
        echo "   e-stop: $ESTOP_HINT (latched); clear with: ros2 service call /estop_reset std_srvs/srv/Trigger"
    fi
fi

# --- 4. arm ----------------------------------------------------------------
if [ -z "$ARM_MODE" ]; then
    say "not arming. The bridge drops every /robot_cmd until start_control is called"
    echo "   arm with: $0 --arm-from-measured --robot-is-clear"
    log_hint
    exit 0
fi

if [ "$ARM_MODE" = "measured" ]; then
    say "reading the current pose"
    if [ "$DRY_RUN" = 1 ]; then
        printf '   '; printf '%q ' "$HERE/g1-measured-pose.py"; printf '\n'
        ARM_VALUES="<$NUM_JOINT measured values>"
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
echo "success=false: latched (/estop_reset), the remote is off, or a wrong length."
echo "The first /robot_cmd cuts the ramp short, so arm before a client streams."
log_hint
