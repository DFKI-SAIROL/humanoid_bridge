#!/usr/bin/env bash
# Run on the user's computer. Fetch pinned sources; PC4 needs no GitHub access.
set -euo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == '--help' ]]; then
  echo '用法：bash deploy_pc4.sh [unitree@192.168.123.164]'
  echo '只下载、复制、编译到 PC4 的 ~/livox_ws，不改网络、不启动驱动。SSH 密码在你的终端输入。'
  exit 0
fi
pc4_host="${1:-unitree@192.168.123.164}"
[[ "$pc4_host" =~ ^[a-zA-Z0-9_.-]+@[a-zA-Z0-9_.-]+$ ]] || { echo 'SSH 目标格式应为 user@host' >&2; exit 1; }
for command_name in git tar scp ssh; do
  command -v "$command_name" >/dev/null || { echo "缺少工具：$command_name" >&2; exit 1; }
done
# Fail before downloads if a workspace already exists; never overwrite existing work.
ssh "$pc4_host" 'test ! -e "$HOME/livox_ws"' || {
  echo '无法登录 PC4，或 ~/livox_ws 已存在。已有目录不会被覆盖。' >&2
  exit 1
}
temporary="$(mktemp -d /tmp/h1-mid360-deploy.XXXXXX)"
trap 'rm -rf "$temporary"' EXIT
fetch_revision() {
  local url="$1" revision="$2" destination="$3"
  git init -q "$destination"
  git -C "$destination" remote add origin "$url"
  git -C "$destination" config remote.origin.promisor true
  git -C "$destination" config remote.origin.partialclonefilter blob:none
  git -C "$destination" fetch --filter=blob:none --depth 1 origin "$revision"
  [[ "$(git -C "$destination" rev-parse FETCH_HEAD)" == "$revision" ]]
}
driver_revision=d4280520f7fe2bc975bd8dce830e7e5d2685035d
sdk_revision=6a940156dd7151c3ab6a52442d86bc83613bd11b
fetch_revision https://github.com/UV-Lab/LIO-SAM_MID360_ROS2_PKG.git "$driver_revision" "$temporary/driver"
fetch_revision https://github.com/Livox-SDK/Livox-SDK2.git "$sdk_revision" "$temporary/sdk"
stage="$temporary/stage"
mkdir -p "$stage/src/livox_ros_driver2" "$stage/third_party/Livox-SDK2" "$stage/scripts" "$stage/config"
git -C "$temporary/driver" archive FETCH_HEAD ros2/src/livox_ros_driver2 | tar -x --strip-components=3 -C "$stage/src/livox_ros_driver2"
git -C "$temporary/sdk" archive FETCH_HEAD | tar -x -C "$stage/third_party/Livox-SDK2"
touch "$stage/third_party/COLCON_IGNORE"
cp "$script_dir/install_pc4.sh" "$script_dir/run_mid360.sh" "$script_dir/configure_mid360.py" "$stage/scripts/"
cat > "$stage/SOURCES.txt" <<SOURCES
Driver: https://github.com/UV-Lab/LIO-SAM_MID360_ROS2_PKG
Commit: $driver_revision
Subdirectory: ros2/src/livox_ros_driver2
SDK: https://github.com/Livox-SDK/Livox-SDK2
Tag: v1.2.5
Commit: $sdk_revision
No network addresses have been configured. No driver has been started.
SOURCES
tar -czf "$temporary/sources.tar.gz" -C "$stage" .
remote_archive="/tmp/h1-mid360-${temporary##*.}.tar.gz"
scp "$temporary/sources.tar.gz" "$pc4_host:$remote_archive"
ssh "$pc4_host" bash -s -- "$remote_archive" <<'REMOTE'
set -euo pipefail
archive="$1"
trap 'rm -f "$archive"' EXIT
# mkdir deliberately fails for an existing directory, including a concurrent deployment.
mkdir "$HOME/livox_ws"
tar -xzf "$archive" -C "$HOME/livox_ws"
bash "$HOME/livox_ws/scripts/install_pc4.sh"
REMOTE
