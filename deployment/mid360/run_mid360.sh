#!/usr/bin/env bash
# Explicit sensor driver start only. No body/hand command interfaces.
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
workspace="$PWD"
[[ -f config/MID360.json ]] || { echo '请先运行 scripts/configure_mid360.py 生成实际 IP 配置。' >&2; exit 1; }
[[ -f install/setup.bash ]] || { echo '请先运行 scripts/install_pc4.sh 编译。' >&2; exit 1; }
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export LD_LIBRARY_PATH="$workspace/third_party/install/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -p "user_config_path:=$workspace/config/MID360.json" \
  -p xfer_format:=0 -p multi_topic:=0 -p data_src:=0 \
  -p publish_freq:=10.0 -p output_data_type:=0 -p frame_id:=livox_frame
