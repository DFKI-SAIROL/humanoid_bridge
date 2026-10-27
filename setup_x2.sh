SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

source "$WORKSPACE_DIR/install/local_setup.bash" || return 1

# Use Fast DDS for X2 mode-service communication.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "Sourced X2 workspace: $WORKSPACE_DIR/install"
