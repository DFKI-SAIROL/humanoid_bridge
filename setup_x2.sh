SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

# Load the ROS 2 environment and X2 workspace
source "$(dirname "$WORKSPACE_DIR")/activate_ros2.sh" || return 1
source "$WORKSPACE_DIR/install_x2/local_setup.bash" || return 1

echo "Sourced X2 workspace: $WORKSPACE_DIR/install_x2"
