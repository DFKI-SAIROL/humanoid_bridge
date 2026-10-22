#!/usr/bin/env bash
# Starts G1_bridge on broken copies of G1_config.yaml: each must refuse to load. No hardware. Exit 1 on failure.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$HERE/loopback_dds.sh"
PKG="$(cd "$HERE/../.." && pwd)"

command -v ros2 >/dev/null || { echo "source the ROS workspace first"; exit 1; }

python3 "$HERE/drive_config.py" "$PKG/params/G1_config.yaml"
