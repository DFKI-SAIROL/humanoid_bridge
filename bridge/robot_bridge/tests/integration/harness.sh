# shellcheck shell=bash
# Sourced by every run_fake_*_test.sh once HERE is set: loopback DDS, paths, process and failure helpers.
# shellcheck disable=SC1091
source "$HERE/loopback_dds.sh"
# The package root, not the repo root: this keeps working if the repo is rearranged around it.
PKG="$(cd "$HERE/../.." && pwd)"
PARAMS="$PKG/params/G1_config.yaml"
# shellcheck disable=SC2034  # used by the sourcing script
OUT="$(mktemp -d)"
export PYTHONPATH="$PKG:$HERE:${PYTHONPATH:-}"

require_ros() { command -v ros2 >/dev/null || { echo "source the ROS workspace first"; exit 1; }; }

# `ros2 run` is a launcher: killing it alone orphans G1_bridge, which keeps holding /lowcmd. Kill the group.
kill_group() { if [ -n "$1" ]; then kill -9 -- "-$1" 2>/dev/null || true; fi; }
kill_pid() { if [ -n "$1" ]; then kill -9 "$1" 2>/dev/null || true; fi; }

# start_bridge <log> [params]: G1_bridge in its own process group; its pid is $!.
start_bridge() {
    setsid ros2 run robot_bridge G1_bridge --ros-args -r __node:=robot_bridge \
        --params-file "${2:-$PARAMS}" > "$1" 2>&1 &
}

# bridge_up <log>: exit 1 unless G1_bridge logged its robot type.
bridge_up() {
    if ! grep -q "G1 type:" "$1"; then
        echo "bridge did not come up:"; tail -20 "$1"; exit 1
    fi
}

# or_tail <log> <command...>: on failure, the log's last 30 lines and exit 1.
or_tail() {
    local log="$1"; shift
    if ! "$@"; then
        echo; echo "bridge log, last 30 lines:"; tail -30 "$log"; exit 1
    fi
}

# verdict <name> PASS|FAIL [detail]: one check line, in the drivers' format.
verdict() { printf '%-44s %s%s\n' "$1" "$2" "${3:+  $3}"; }
