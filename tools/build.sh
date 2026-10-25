#!/usr/bin/env bash
# colcon build this workspace with the defaults that actually work.
#
# Needs: ROS 2 Humble sourced, and this clone under <workspace>/src/.
# Run tools/install-build-deps.sh first on a fresh host.
#
# What this exists to get right:
#   - colcon never removes what it installed, so a renamed or deleted node keeps
#     being offered by `ros2 run` until its build/ and install/ trees go.
#     --clean <pkg> is that.
#
# Safe to re-run.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"

WORKSPACE=""
CLEAN_PKGS=()
CLEAN_ALL=0

usage() {
    cat <<'EOF'
Usage: tools/build.sh [options] [-- colcon args...]

colcon build this workspace, T1_bridge included (no Booster SDK needed).

Options:
  --workspace <path>  colcon workspace root (default: the dir above src/)
  --t1                accepted and ignored: T1_bridge always builds
  --clean <pkg>       drop build/<pkg> and install/<pkg> first; repeatable
  --clean-all         drop build/, install/ and log/ first
  -h, --help          this text

Anything after -- goes to colcon, e.g.
  tools/build.sh -- --packages-select robot_bridge

Then: tools/test.sh
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --workspace) WORKSPACE="${2:?--workspace needs a path}"; shift ;;
        --t1)        ;;
        --clean)     CLEAN_PKGS+=("${2:?--clean needs a package name}"); shift ;;
        --clean-all) CLEAN_ALL=1 ;;
        -h|--help)   usage; exit 0 ;;
        --)          shift; break ;;
        *)           echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
    shift
done

# --- workspace -------------------------------------------------------------
if [ -z "$WORKSPACE" ]; then
    # Walk up for the dir holding src/, rather than assuming ~/bridge_ws.
    WORKSPACE="$REPO"
    while [ "$WORKSPACE" != "/" ] && [ ! -d "$WORKSPACE/src" ]; do
        WORKSPACE="$(dirname "$WORKSPACE")"
    done
    if [ ! -d "$WORKSPACE/src" ]; then
        echo "no colcon workspace above $REPO: this clone must live under <workspace>/src/." >&2
        echo "    mkdir -p ~/bridge_ws/src && ln -s $REPO ~/bridge_ws/src/humanoid_bridge" >&2
        echo "or pass --workspace <path>." >&2
        exit 1
    fi
fi
[ -d "$WORKSPACE" ] || { echo "no such workspace: $WORKSPACE" >&2; exit 1; }

# --- preflight -------------------------------------------------------------
command -v colcon >/dev/null || {
    echo "colcon not on PATH. Source ROS 2 and install it:" >&2
    echo "    source /opt/ros/humble/setup.bash && tools/install-build-deps.sh" >&2
    exit 1; }

if [ -z "${ROS_DISTRO:-}" ]; then
    echo "ROS_DISTRO is not set: source /opt/ros/humble/setup.bash first" >&2
    exit 1
fi

# thirdparty/unitree/*/CMakeLists.txt require this and their package.xml never
# declares it, so rosdep does not pull it and CMake fails with a bare
# "Could not find a package configuration file".
if ! [ -d "/opt/ros/$ROS_DISTRO/share/rosidl_generator_dds_idl" ] \
   && ! ros2 pkg prefix rosidl_generator_dds_idl >/dev/null 2>&1; then
    echo "rosidl_generator_dds_idl is missing; unitree_go/hg/api will not configure." >&2
    echo "    sudo apt-get install -y ros-$ROS_DISTRO-rosidl-generator-dds-idl" >&2
    echo "    (or: tools/install-build-deps.sh)" >&2
    exit 1
fi

# --- clean -----------------------------------------------------------------
if [ "$CLEAN_ALL" = 1 ]; then
    rm -rf "${WORKSPACE:?}/build" "${WORKSPACE:?}/install" "${WORKSPACE:?}/log"
    echo "cleared the whole workspace"
fi
for pkg in ${CLEAN_PKGS+"${CLEAN_PKGS[@]}"}; do
    rm -rf "${WORKSPACE:?}/build/$pkg" "${WORKSPACE:?}/install/$pkg"
    echo "cleared $pkg"
done

# --- build -----------------------------------------------------------------
echo "workspace: $WORKSPACE"
cd "$WORKSPACE"
# ON, not the default: a build dir from before keeps a cached OFF. A later --cmake-args replaces this.
colcon build --cmake-args -DBUILD_BOOSTER_T1=ON "$@"

echo
echo "done. Source it, then test:"
echo "    source $WORKSPACE/install/setup.bash"
echo "    tools/test.sh"
