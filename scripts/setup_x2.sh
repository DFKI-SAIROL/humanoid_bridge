# shellcheck shell=bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Walk up for the X2 workspace (built into install_x2/), as setup_booster.sh does.
WORKSPACE_DIR="$SCRIPT_DIR"
while [ "$WORKSPACE_DIR" != "/" ] && [ ! -f "$WORKSPACE_DIR/install_x2/local_setup.bash" ]; do
    WORKSPACE_DIR="$(dirname "$WORKSPACE_DIR")"
done
if [ ! -f "$WORKSPACE_DIR/install_x2/local_setup.bash" ]; then
    echo "no install_x2/ workspace above $SCRIPT_DIR -- build it first" >&2
    return 1 2>/dev/null || exit 1
fi

source "$WORKSPACE_DIR/install_x2/local_setup.bash"

# Use Fast DDS for X2 mode-service communication.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "Sourced X2 workspace: $WORKSPACE_DIR/install_x2"
