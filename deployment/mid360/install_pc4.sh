#!/usr/bin/env bash
# Build only: no network changes, service changes, driver start or apt/sudo calls.
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
workspace="$PWD"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo '需要 PC4 的系统 ROS 2 Humble：/opt/ros/humble/setup.bash' >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
set -u
for command_name in cmake colcon c++ python3 pkg-config; do
  command -v "$command_name" >/dev/null || { echo "缺少工具：$command_name" >&2; exit 1; }
done
if ! pkg-config --exists pcl_common || ! ros2 pkg prefix ament_cmake_auto >/dev/null 2>&1 || ! ros2 pkg prefix pcl_conversions >/dev/null 2>&1 || ! ros2 pkg prefix rosidl_default_generators >/dev/null 2>&1; then
  echo '缺少编译依赖。请在 PC4 可联网时安装，然后重运行本脚本：' >&2
  echo 'sudo apt-get install --no-upgrade build-essential cmake pkg-config python3-colcon-common-extensions libpcl-dev ros-humble-ament-cmake-auto ros-humble-pcl-conversions ros-humble-rosidl-default-generators' >&2
  exit 1
fi
# Upstream probes APR without REQUIRED; missing APR alone must not block a build.
if ! pkg-config --exists apr-1; then
  echo "APR 未安装：上游为可选探测，继续使用现有依赖编译。"
fi
jobs="${MID360_BUILD_JOBS:-2}"
[[ "$jobs" =~ ^[1-9][0-9]*$ ]] || { echo 'MID360_BUILD_JOBS 必须为正整数' >&2; exit 1; }
prefix="$workspace/third_party/install"
cmake -S third_party/Livox-SDK2 -B build/sdk -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$prefix"
cmake --build build/sdk --target livox_lidar_sdk_shared livox_lidar_sdk_static --parallel "$jobs"
# SDK installation is restricted to this workspace; no /usr/local writes.
cmake --install build/sdk
cp src/livox_ros_driver2/package_ROS2.xml src/livox_ros_driver2/package.xml
CMAKE_BUILD_PARALLEL_LEVEL="$jobs" colcon build --packages-select livox_ros_driver2 --parallel-workers 1 --cmake-args \
  -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble -DBUILD_TESTING=OFF \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$prefix;/opt/ros/humble" \
  -DLIVOX_LIDAR_SDK_LIBRARY="$prefix/lib/liblivox_lidar_sdk_shared.so" \
  -DLIVOX_LIDAR_SDK_INCLUDE_DIR="$prefix/include"
printf '\n编译完成：%s\n未配置网络，未启动雷达。下一步用 scripts/configure_mid360.py 填写实际 IP。\n' "$workspace"
