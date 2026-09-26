SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

source "$WORKSPACE_DIR/install_x2/local_setup.bash" || return 1

echo "Sourced X2 workspace: $WORKSPACE_DIR/install_x2"
