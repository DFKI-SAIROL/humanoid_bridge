# H1-2 安装与启动

系统：Ubuntu 22.04；PC4 使用 `/opt/ros/humble`；电脑运行可视化，PC4 运行手部、雷达和头部相机驱动。

| 项目 | 配置 |
| --- | --- |
| 工作空间 | `~/sariol_ws` |
| 电脑有线地址 | `192.168.123.99/24` |
| 电脑有线网卡 | `enp0s31f6`，按实际网卡名替换 |
| PC4 | `unitree@192.168.123.164`，网卡 `eth0` |
| MID360 | `192.168.124.20`，按雷达出厂地址填写 |
| PC4 雷达接收地址 | `192.168.124.164/24` |
| ROS 域 | `0` |

雷达与 PC4 接收地址使用同一网段；更改地址时，同步替换下文网口配置与雷达 JSON 中的地址。

`bash` 块为命令，`text` 块为关键输出示例；`#` 为注释。

## 1. 接线与网络

1. 电脑网线连接机器人以太网口，机器人开机。
2. 灵巧手的 USB 串口转换器接 PC4，设备名使用 `/dev/ttyUSB0`。
3. 沿头部、颈部找到深度相机 USB 引出线，从原接入端拔下，插入 **PC4 USB 3.0 接口**或其 USB 3.0 拓展坞。
4. MID360 保持原装供电和以太网接线。

PC4 USB 位置见[官方中文接口图](https://doc-cdn.unitree.com/static/2025/4/18/a1f96ffec36647ab9f456cd143f14c3b_2482x3509.png)：右侧四口 USB 组，上排①②连接 PC4，下排③④连接 PC5。

**电脑终端（首次配置；Wi-Fi 联网）：**

```bash
ip -br link
sudo nmcli connection add type ethernet con-name h1-2 \
  ifname enp0s31f6 ipv4.method manual \
  ipv4.addresses 192.168.123.99/24 ipv4.never-default yes
sudo nmcli connection up h1-2
```

```text
Connection 'h1-2' (...) successfully added.
Connection successfully activated (...)
# 有线连接已启用
```

PC4 保留原厂网络配置；雷达接收网口调整见文末。

## 2. 安装驱动

### 2.1 电脑环境与可视化

在仓库所在的电脑终端执行。源码放在 `~/sariol_ws/src/humanoid_bridge`。

安装 [Miniforge](https://github.com/conda-forge/miniforge)，已有 Conda 时直接创建 ROS 环境：

```bash
curl -fL https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh \
  -o /tmp/Miniforge3-Linux-x86_64.sh
bash /tmp/Miniforge3-Linux-x86_64.sh -b -p "$HOME/miniforge3"
rm /tmp/Miniforge3-Linux-x86_64.sh
source ~/miniforge3/etc/profile.d/conda.sh
```

```text
installation finished.
```

创建 ROS 环境、编译消息、安装查看器并下载官方 H1-2 模型：

```bash
cd ~/sariol_ws/src/humanoid_bridge/visualization
conda env create -n ros2_humble -f environment.ros2.yaml
conda activate ros2_humble

cd ~/sariol_ws
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build \
  --packages-up-to unitree_go unitree_hg bridge_interface \
  --parallel-workers 1 --cmake-args -DBUILD_TESTING=OFF

cd ~/sariol_ws/src/humanoid_bridge/visualization
python -m pip install -e .
h1-2-models fetch
```

```text
# 环境创建完成
# 激活后提示符带 (ros2_humble)
Summary: 3 packages finished [...]
Successfully installed ... h1-2-viewer-0.2.0 ...
/home/<用户名>/.cache/h1_2_viewer/models/...
```

### 2.2 PC4 下载环境

PC4 通过 SSH 借用电脑网络下载，日常启动不需要外网。

**电脑安装终端：**

```bash
ssh -o ExitOnForwardFailure=yes \
  -R 127.0.0.1:1080 unitree@192.168.123.164
```

```text
unitree@192.168.123.164's password:  # 输入 PC4 密码，不回显
unitree@unitree-h1-2-pc4:~$         # 已登录；保持此连接
```

**PC4 安装终端（后续安装均在此执行）：**

```bash
export http_proxy=socks5h://127.0.0.1:1080
export https_proxy="$http_proxy"

sudo apt -o Acquire::http::Proxy="$http_proxy" \
  -o Acquire::https::Proxy="$https_proxy" update
sudo apt -o Acquire::http::Proxy="$http_proxy" \
  -o Acquire::https::Proxy="$https_proxy" install \
  git curl build-essential cmake pkg-config libboost-program-options-dev \
  python3-colcon-common-extensions libpcl-dev \
  ros-humble-ament-cmake-auto ros-humble-pcl-conversions \
  ros-humble-rosidl-default-generators ros-humble-rmw-cyclonedds-cpp \
  ros-humble-realsense2-camera
```

```text
Reading package lists... Done
Do you want to continue? [Y/n] y   # 输入 y
Setting up ros-humble-realsense2-camera (...)
unitree@unitree-h1-2-pc4:~$         # 安装结束
```

`apt update` 出现时间或签名错误时，按文末对应命令处理后重试。

### 2.3 灵巧手驱动

下载：[Unitree SDK2](https://github.com/unitreerobotics/unitree_sdk2)、[DFX Inspire Hand Service](https://github.com/unitreerobotics/dfx_inspire_service)。

**PC4 安装终端：**

```bash
cd ~
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd ~/unitree_sdk2
git checkout d1abf001220d3e5f59e00a0c4aaa595e00089a4a
cmake -S . -B build -DBUILD_EXAMPLES=OFF -DCMAKE_BUILD_TYPE=Release
sudo cmake --install build
sudo ldconfig

cd ~
git clone https://github.com/unitreerobotics/dfx_inspire_service.git
cd ~/dfx_inspire_service
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target inspire_h1 --parallel 2
```

```text
[100%] Built target inspire_h1
```

### 2.4 MID360 驱动

下载：[Livox SDK2 v1.2.5](https://github.com/Livox-SDK/Livox-SDK2/tree/v1.2.5)、[UV-Lab livox_ros_driver2](https://github.com/UV-Lab/LIO-SAM_MID360_ROS2_PKG/tree/d4280520f7fe2bc975bd8dce830e7e5d2685035d/ros2/src/livox_ros_driver2)。

**PC4 安装终端：**准备源码并编译 SDK。

```bash
mkdir -p ~/livox_ws/src ~/livox_ws/third_party
touch ~/livox_ws/third_party/COLCON_IGNORE
git clone --depth 1 --branch v1.2.5 \
  https://github.com/Livox-SDK/Livox-SDK2.git \
  ~/livox_ws/third_party/Livox-SDK2

MID360_SRC=$(mktemp -d /tmp/h1-mid360.XXXXXX)
git clone --filter=blob:none --no-checkout \
  https://github.com/UV-Lab/LIO-SAM_MID360_ROS2_PKG.git "$MID360_SRC"
git -C "$MID360_SRC" sparse-checkout set ros2/src/livox_ros_driver2
git -C "$MID360_SRC" checkout d4280520f7fe2bc975bd8dce830e7e5d2685035d
cp -a "$MID360_SRC/ros2/src/livox_ros_driver2" ~/livox_ws/src/
rm -rf "$MID360_SRC"

cd ~/livox_ws
cmake -S third_party/Livox-SDK2 -B build/sdk \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$HOME/livox_ws/third_party/install"
cmake --build build/sdk --target livox_lidar_sdk_shared livox_lidar_sdk_static --parallel 2
cmake --install build/sdk
```

```text
Built target livox_lidar_sdk_shared
Built target livox_lidar_sdk_static
-- Installing: /home/unitree/livox_ws/third_party/install/lib/...
```

编译 ROS 驱动：

```bash
source /opt/ros/humble/setup.bash
cd ~/livox_ws
cp src/livox_ros_driver2/package_ROS2.xml src/livox_ros_driver2/package.xml
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build \
  --packages-select livox_ros_driver2 \
  --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble \
  -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
  -DLIVOX_LIDAR_SDK_LIBRARY="$HOME/livox_ws/third_party/install/lib/liblivox_lidar_sdk_shared.so" \
  -DLIVOX_LIDAR_SDK_INCLUDE_DIR="$HOME/livox_ws/third_party/install/include"
```

```text
Starting >>> livox_ros_driver2
Finished <<< livox_ros_driver2 [...]
Summary: 1 package finished [...]
# pcap will be disabled 警告不影响在线采集
```

填写雷达与接收地址，外参保持零。修改以下两行地址以匹配接线与网络配置。

```bash
python3 - <<'PY'
import json
from pathlib import Path
lidar_ip = '192.168.124.20'
host_ip = '192.168.124.164'
p = Path.home() / 'livox_ws/src/livox_ros_driver2/config/MID360_config.json'
cfg = json.loads(p.read_text())
cfg['lidar_configs'][0]['ip'] = lidar_ip
cfg['lidar_configs'][0]['extrinsic_parameter'] = dict.fromkeys(
    ('roll', 'pitch', 'yaw', 'x', 'y', 'z'), 0)
for key in ('cmd_data_ip', 'push_msg_ip', 'point_data_ip', 'imu_data_ip'):
    cfg['MID360']['host_net_info'][key] = host_ip
p.write_text(json.dumps(cfg, indent=2) + '\n')
PY
```

```text
unitree@unitree-h1-2-pc4:~/livox_ws$
# 无输出：已保存 MID360_config.json；未修改网卡、未启动雷达
```

### 2.5 头部深度相机驱动

驱动：[RealSense ROS 2](https://github.com/realsenseai/realsense-ros)。已由 2.2 节的 `ros-humble-realsense2-camera` 安装，同时提供相机 SDK 运行库。

**PC4 安装终端：**读取头部相机序列号，记录 `ID_SERIAL_SHORT`。

```bash
udevadm info --query=property --name=/dev/video0 | grep '^ID_SERIAL_SHORT='
```

```text
ID_SERIAL_SHORT=337122072045  # 记录实际输出的序列号
```

## 3. 第一次启动

打开四个终端。终端 1～3 先登录 PC4，再执行对应命令；终端 4 在电脑运行。

```bash
ssh unitree@192.168.123.164
```

```text
unitree@192.168.123.164's password:
unitree@unitree-h1-2-pc4:~$
```

### 终端 1：PC4 灵巧手

```bash
cd ~/dfx_inspire_service/build
sudo ./inspire_h1 -s /dev/ttyUSB0 --network eth0
```

```text
 --- Unitree Robotics ---
  Inspire Hand Controller
# 保持运行；手部反馈：/inspire/state
```

### 终端 2：PC4 MID360

```bash
source /opt/ros/humble/setup.bash
source ~/livox_ws/install/local_setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0"/></Interfaces></General></Domain></CycloneDDS>'
export LD_LIBRARY_PATH="$HOME/livox_ws/third_party/install/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -p user_config_path:="$HOME/livox_ws/src/livox_ros_driver2/config/MID360_config.json" \
  -p xfer_format:=0 -p publish_freq:=10.0 -p frame_id:=livox_frame
```

```text
Init lds lidar success!
livox/imu publish use imu format
livox/lidar publish use PointCloud2 format
# 保持运行；点云：/livox/lidar
```

### 终端 3：PC4 头部相机

将 `相机序列号` 替换为 2.5 节记录的值，保留前面的下划线。

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0"/></Interfaces></General></Domain></CycloneDDS>'

ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=camera camera_name:=head \
  serial_no:=_相机序列号 \
  enable_color:=true enable_depth:=true \
  rgb_camera.color_profile:=640x480x15 \
  depth_module.depth_profile:=640x480x15 \
  align_depth.enable:=true pointcloud.enable:=false
```

```text
Open profile: stream_type: Depth(0), Format: Z16, Width: 640, Height: 480, FPS: 15
Open profile: stream_type: Color(0), Format: RGB8, Width: 640, Height: 480, FPS: 15
RealSense Node Is Up!
# 保持运行；彩色、深度均为 640×480、15Hz
```

### 终端 4：电脑可视化

```bash
source ~/miniforge3/etc/profile.d/conda.sh
conda activate ros2_humble
cd ~/sariol_ws
export ROS_DOMAIN_ID=0
export UNITREE_NETWORK_INTERFACE=enp0s31f6
source src/humanoid_bridge/setup_unitree.sh
cd src/humanoid_bridge/visualization

h1-2-viewer --config h1_2_viewer/config/h1_2_live.yaml \
  --observe-client --serve
```

```text
Set CycloneDDS and network interface enp0s31f6; ...
Sourced workspace: /home/<用户名>/sariol_ws
可视化地址：http://localhost:9090/?url=...
只读观察；按 Ctrl+C 停止可视化，不影响控制 Client。
# 保持运行；复制终端中的完整网址到浏览器打开
```

- 主页面：机器人模型、雷达点云、头部彩色深度点云、头部彩色图像。
- 调试页：身体 IMU、关节状态、深度图和坐标变换。
- 左侧图层：显示或隐藏机器人、雷达、头部深度点云。

```text
# 正常：头部画面、彩色深度点云、雷达点云持续刷新；IMU、关节曲线更新
# 结束：各运行终端按 Ctrl+C；SSH 终端输入 exit
```

## 按需处理

### PC4 时间错误

安装时报 `certificate is not yet valid` 或 `Release file ... is not valid yet` 时，在**电脑另一个终端**同步 PC4 时间，再回到安装终端重试：

```bash
ssh -t unitree@192.168.123.164 \
  "sudo date -u -s '$(date -u +%Y-%m-%dT%H:%M:%SZ)'"
```

```text
# 打印同步后的 UTC 时间，随后返回电脑命令提示符
```

### ROS 签名密钥过期

`apt update` 报 `EXPKEYSIG` 时，在 **PC4 安装终端**更新密钥，再重试 `apt update`：

```bash
curl -fL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /tmp/ros-new.key
sudo install -m 644 /tmp/ros-new.key /usr/share/keyrings/ros-archive-keyring.gpg
rm /tmp/ros-new.key
```

```text
# 密钥更新完成；重新 apt update 后 EXPKEYSIG 消失
```

### 雷达接收地址挂在 br0、数据从 eth0 进入

在 **PC4 终端**将雷达接收地址移到 `eth0`，保留 SSH 地址：

```bash
sudo netplan set 'ethernets.eth0.addresses=[192.168.123.164/24,192.168.124.164/24]'
sudo netplan set 'bridges.br0.addresses=null'
sudo netplan apply
```

```text
unitree@unitree-h1-2-pc4:~$
# 无输出：eth0 配置已生效，重启后保留
```
