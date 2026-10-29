# PC4 MID360 驱动部署

架构：MID360 → PC4 的 Livox SDK2 / ROS 2 驱动 → `/livox/lidar` → 电脑上的只读可视化。
与全身控制 Client、Inspire 手部程序独立。当前仅准备部署，未验证这台机器的雷达 IP 或实测点云。

## 部署到 PC4

在有 GitHub 网络的开发电脑运行：

```bash
cd /path/to/humanoid_bridge
bash deployment/mid360/deploy_pc4.sh
```

默认 SSH 目标为 `unitree@192.168.123.164`，可作为第一个参数修改。按终端提示输入 PC4 的 SSH 密码。
脚本下载固定版本源码、复制到 PC4 的 `~/livox_ws`，然后编译，不启动驱动、不改网络、不停止已有服务。
本机下载及传输包位于 /tmp，正常结束或失败退出后清理；PC4 解包后删除传输包。
如果 PC4 已存在 `~/livox_ws`，脚本拒绝覆盖。若首次因依赖缺失而中断，按提示安装依赖后，在 PC4 继续：

```bash
bash ~/livox_ws/scripts/install_pc4.sh
```

需要 PC4 的系统 `/opt/ros/humble`，不使用开发电脑的 Conda 环境编译。
上游 CMake 将 APR 作为非 REQUIRED 探测；缺少 APR 不再阻止编译。优先使用现有依赖，不要求为了升级已安装的 ROS 包而联网。
如果 apt 报 Temporary failure resolving，不代表编译器或源码出错，也不要改正在承载 SSH 的网络；先用更新后的安装脚本尝试离线编译。
SDK 源码、许可证、版本记录保留在工作空间，SDK 安装到 `~/livox_ws/third_party/install`，不写 /usr/local。
默认编译并行度为 2，可通过 MID360_BUILD_JOBS 设置。没有调用原工程会清空工作空间的 build.sh。

## 配置与显式启动

只需要确认这台机器的雷达 IP 和 PC4 接收网口 IP。下面参数为占位符，请替换为实际地址，不要直接复制旧 `.1.162/.1.50` 配置。

```bash
python3 ~/livox_ws/scripts/configure_mid360.py \
  --lidar-ip ACTUAL_LIDAR_IP --host-ip ACTUAL_PC4_RECEIVER_IP
```

脚本检查接收地址已在 PC4 本机网口上，生成 `~/livox_ws/config/MID360.json`；不会配置网口、设置雷达 IP 或改变其持久网络参数。
旧配置不会默认覆盖，确认替换时显式加 --force。生成配置不证明网络可达。
确认网络和实际地址后，再显式启动：

```bash
bash ~/livox_ws/scripts/run_mid360.sh
```

SDK 启动时会向雷达发送配置/采集请求，并设置数据接收目标；不要与另一套采集程序同时争用同一雷达。
启动参数固定为 xfer_format=0（PointCloud2）、multi_topic=0、10 Hz，frame_id=livox_frame。
SDK 外参保持零，防止和机器人 URDF / TF 重复应用安装变换。

## 验证与可视化

在 PC4 另开终端（驱动保持运行）：

```bash
source /opt/ros/humble/setup.bash
source ~/livox_ws/install/local_setup.bash
ros2 topic hz /livox/lidar
```

频率只能证明有消息；还需确认消息 width×height 大于 0、xyz 有限，再验证实际点云。
电脑沿用原 CycloneDDS 有线环境，可视化已有 `/livox/lidar` 的 PointCloud2 只读订阅：

```bash
h1-2-viewer --observe-client --serve --language zh
```

三维融合仍需要 livox_frame 到机器人 lidar_link 的真实坐标关系。不能仅凭重命名假定完全对齐；未有 TF 时平台会等待。
本次部署不修改机器人网络、控制代码或查看器的标定配置，也不部署 LIO-SAM/GTSAM。

## 来源

- UV-Lab/LIO-SAM_MID360_ROS2_PKG，提交 d4280520f7fe2bc975bd8dce830e7e5d2685035d，仅取 ros2/src/livox_ros_driver2。
- Livox-SDK/Livox-SDK2，v1.2.5，提交 6a940156dd7151c3ab6a52442d86bc83613bd11b。
- https://github.com/UV-Lab/LIO-SAM_MID360_ROS2_PKG
- https://github.com/Livox-SDK/Livox-SDK2
