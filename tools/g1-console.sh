#!/usr/bin/env bash
# Start the robot console for the G1 lab, after checking that what runs matches
# this source tree:
#
#   robot_console build   install/ differs from the source -> rebuilt here
#   G1_bridge build       a bridge source file is newer than the binary
#   running G1_bridge     started before its binary was built, from another
#                         workspace, or more than one (always refused)
#   Jetson ~/g1_camera    a file differs from robot_stacks/g1/g1_camera
#
# Run it in a foreground tmux pane: it execs the console, so Ctrl-C reaches it.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd -P)"

WS="${BRIDGE_WS:-$HOME/bridge_ws_estop}"
LAB_REPO="${LAB_REPO:-$HOME/github/humanoid-locoman-vla}"
CONFIG=""
HOST=""
G1_SSH="${G1_SSH:-g1}"
G1_CAMERA_DIR="${G1_CAMERA_DIR:-g1_camera}"   # relative to the Jetson user's home
SYNC_JETSON=0
ALLOW_MISMATCH=0
CHECK_ONLY=0

usage() {
    cat <<'EOF'
Usage: tools/g1-console.sh [options] [-- <robot_console args>]

Check versions, then start robot_console for the G1 lab.

Options:
  --workspace <ws>    colcon workspace the robot runs (default $BRIDGE_WS or ~/bridge_ws_estop)
  --config <yaml>     console config (default $LAB_REPO/benches/g1/console.yaml)
  --host <ip>         bind address (default: the source IP of the default route)
  --sync-jetson       rsync robot_stacks/g1/g1_camera to the Jetson if it differs
  --allow-mismatch    start even if a version check fails (two bridges: still refused)
  --check             run the checks only, start nothing
  -h, --help          this text

Environment: LAB_REPO (~/github/humanoid-locoman-vla), LAB_STORE (~/psi-home),
BRIDGE_IFACE (enp4s0), G1_SSH (g1), G1_CAMERA_DIR (g1_camera).
ROBOT_CONSOLE_CONFIG_DIR is $LAB_REPO/benches/g1 even with --config, so the
processes run from the lab clone.
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --workspace) WS="${2:?--workspace needs a path}"; shift ;;
        --config)    CONFIG="${2:?--config needs a path}"; shift ;;
        --host)      HOST="${2:?--host needs an address}"; shift ;;
        --sync-jetson) SYNC_JETSON=1 ;;
        --allow-mismatch) ALLOW_MISMATCH=1 ;;
        --check)     CHECK_ONLY=1 ;;
        -h|--help)   usage; exit 0 ;;
        --)          shift; break ;;
        *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
    shift
done

say()  { printf '== %s\n' "$*"; }
ok()   { printf '   ok    %s\n' "$*"; }
warn() { printf '   WARN  %s\n' "$*"; }
FAILED=()
fail() { printf '   FAIL  %s\n' "$*"; FAILED+=("$1"); }

export ROBOT_CONSOLE_CONFIG_DIR="${ROBOT_CONSOLE_CONFIG_DIR:-$LAB_REPO/benches/g1}"
CONFIG="${CONFIG:-$ROBOT_CONSOLE_CONFIG_DIR/console.yaml}"
[ -f "$CONFIG" ] || { echo "no console config at $CONFIG (pass --config)" >&2; exit 1; }
if [ -z "$HOST" ]; then
    HOST="$(ip route get 1.1.1.1 2>/dev/null | sed -n 's/.* src \([0-9.]*\).*/\1/p')"
    HOST="${HOST:-127.0.0.1}"
fi

# --- the workspace must build this clone ------------------------------------
WS_CLONE="$WS/src/humanoid_bridge"
if [ "$(cd "$WS_CLONE" 2>/dev/null && pwd -P)" != "$REPO" ]; then
    echo "$WS_CLONE is not this clone ($REPO)." >&2
    echo "Run the script from the clone the workspace builds, or pass --workspace." >&2
    exit 1
fi
say "source $(git -C "$REPO" describe --always --dirty) ($REPO)"
say "workspace $WS"

# --- environment --------------------------------------------------------------
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WS_CLONE/scripts/setup_unitree.sh" >/dev/null
set -u
export BRIDGE_CLONE="$WS_CLONE"
export LAB_STORE="${LAB_STORE:-$HOME/psi-home}"

# --- robot_console build ------------------------------------------------------
say "robot_console build"
CONSOLE_SRC="$REPO/bridge/robot_console/robot_console"
console_installed() {
    local site
    site="$(ls -d "$WS"/install/robot_console/lib/python3*/site-packages/robot_console 2>/dev/null | head -1)"
    [ -n "$site" ] && diff -rq -x __pycache__ "$CONSOLE_SRC" "$site" >/dev/null
}
if console_installed; then
    ok "install matches the source"
elif [ "$CHECK_ONLY" = 1 ]; then
    fail "install differs from the source (a start rebuilds it)"
elif pgrep -f "install/robot_console/lib/robot_console/robot_console" >/dev/null; then
    fail "install differs from the source, and a console runs: stop it, then re-run"
else
    echo "   install differs from the source: colcon build --packages-select robot_console"
    (cd "$WS" && colcon build --packages-select robot_console >/dev/null)
    if console_installed; then ok "rebuilt"; else fail "still differs after the rebuild"; fi
fi

# --- G1_bridge build ----------------------------------------------------------
say "G1_bridge build"
BRIDGE_BIN="$WS/install/robot_bridge/lib/robot_bridge/G1_bridge"
if [ ! -x "$BRIDGE_BIN" ]; then
    fail "no binary at $BRIDGE_BIN"
else
    mapfile -t newer < <(find "$REPO/bridge/robot_bridge/src" "$REPO/bridge/robot_bridge/include" \
        "$REPO/bridge/robot_bridge/CMakeLists.txt" "$REPO/bridge/robot_bridge/package.xml" \
        "$REPO/bridge/bridge_interface" -type f -newer "$BRIDGE_BIN" ! -path '*/__pycache__/*' \
        | sed "s|^$REPO/||")
    if [ "${#newer[@]}" -eq 0 ]; then
        ok "binary is newer than every bridge source file"
    else
        fail "source changed after the build (${#newer[@]} files, e.g. ${newer[0]}): rebuild $WS"
    fi
fi

# --- running G1_bridge --------------------------------------------------------
say "running G1_bridge"
mapfile -t pids < <(pgrep -x G1_bridge || true)
DUPLICATE=0
if [ "${#pids[@]}" -eq 0 ]; then
    warn "none runs. Start it: tools/g1-bringup.sh"
elif [ "${#pids[@]}" -gt 1 ]; then
    DUPLICATE=1
    printf '   FAIL  %s G1_bridge processes; each writes /lowcmd. Stop the extra one:\n' "${#pids[@]}"
    for p in "${pids[@]}"; do
        printf '         pid %s, started %s, parent: %s\n' "$p" \
            "$(ps -o lstart= -p "$p")" "$(ps -o args= -p "$(ps -o ppid= -p "$p" | tr -d ' ')" | cut -c1-60)"
    done
fi
if [ -x "$BRIDGE_BIN" ]; then
    bin_mtime="$(stat -c %Y "$BRIDGE_BIN")"
    for p in "${pids[@]}"; do
        exe="$(readlink "/proc/$p/exe" || true)"
        started=$(( $(date +%s) - $(ps -o etimes= -p "$p" | tr -d ' ') ))
        if [ "$exe" != "$BRIDGE_BIN" ]; then
            fail "pid $p runs $exe, not this workspace's binary"
        elif [ "$started" -lt "$bin_mtime" ]; then
            fail "pid $p started before the binary was built: restart the bridge"
        else
            ok "pid $p runs the current binary"
        fi
    done
fi

# --- Jetson g1_camera ---------------------------------------------------------
say "Jetson $G1_SSH:~/$G1_CAMERA_DIR"
CAMERA_SRC="$REPO/robot_stacks/g1/g1_camera"
jetson_check() {   # 0 same, 1 differs (prints the files), 255 unreachable
    (cd "$CAMERA_SRC" && git ls-files -z | xargs -0 sha256sum) \
        | ssh -o BatchMode=yes -o ConnectTimeout=3 "$G1_SSH" \
            "cd $G1_CAMERA_DIR 2>/dev/null && sha256sum --quiet -c - 2>&1 || exit 1" 2>/dev/null
}
set +e; diffs="$(jetson_check)"; rc=$?; set -e
if [ "$rc" -eq 1 ] && [ "$SYNC_JETSON" = 1 ]; then
    echo "   differs: rsync to the Jetson"
    rsync -a --exclude __pycache__ --exclude '*.egg-info' "$CAMERA_SRC/" "$G1_SSH:$G1_CAMERA_DIR/"
    set +e; diffs="$(jetson_check)"; rc=$?; set -e
fi
case "$rc" in
    0)   ok "matches robot_stacks/g1/g1_camera" ;;
    255) warn "unreachable over ssh; the camera process will fail" ;;
    *)   fail "differs from robot_stacks/g1/g1_camera (re-run with --sync-jetson):"
         printf '%s\n' "$diffs" | grep -v '^sha256sum:' | sed 's/^/         /' | head -10 ;;
esac

# --- verdict ------------------------------------------------------------------
if [ "$DUPLICATE" = 1 ]; then
    echo "refused: more than one G1_bridge runs." >&2
    exit 1
fi
if [ "${#FAILED[@]}" -gt 0 ] && [ "$ALLOW_MISMATCH" != 1 ]; then
    echo "refused: ${#FAILED[@]} check(s) failed. Repair them, or pass --allow-mismatch." >&2
    exit 1
fi
[ "$CHECK_ONLY" = 1 ] && exit 0

say "robot_console on http://$HOST:8080 ($CONFIG)"
exec ros2 run robot_console robot_console --host "$HOST" --config "$CONFIG" "$@"
