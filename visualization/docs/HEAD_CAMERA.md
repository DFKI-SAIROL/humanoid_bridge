# 头部相机

## 官方接口与本机结果

宇树 [H1-2 官方说明](https://support.unitree.com/home/en/H1_developer/About_H1-2) 标注头部深度相机为 D435。原厂图像传输、RealSense 深度采集是两条不同的链路，不能用彩色视频推导真实深度或相机内参。
宇树也公开了用于多相机采集的 [TeleImager](https://github.com/unitreerobotics/teleimager)，支持 RealSense、ZMQ 和 WebRTC；本机已存在原厂视频发布器，无需为了显示 RGB 再安装一套采集服务。

2026-10-29 本机只读发现 `/frontvideostream`，类型广告为 `unitree_go/msg/Go2FrontVideoData`。公开 [Unitree ROS 消息](https://github.com/unitreerobotics/unitree_ros2/blob/master/cyclonedds_ws/src/unitree/unitree_go/msg/Go2FrontVideoData.msg) 是 timestamp + 三个分辨率字节数组；当前 H1 固件实际采样则为 CDR1 timestamp + height + H.264 bytes，360/720 两种流交替发布。两者不匹配，不能直接用公开消息定义反序列化当前流。

native_video.py 使用只读 raw 订阅，按实测封装解析，同时兼容公开三数组封装；不修改 unitree_go 消息包。PyAV 连续解码 H.264，再只保留最新 RGB。所有参考帧都参与解码，限流在解码之后，避免简单丢弃 P 帧破坏图像。加入流时可能先等下一个关键帧；不会向 videohub 发请求或启动／停止原厂服务。

实测原厂流解出 640×360 图像，7 秒采样解码 104 帧；完整查看器 6 秒联调输出 23 批头部画面和 56 批身体／IMU，未报告相机错误。该次没有收到雷达和手部反馈，未据此声称所有传感器同时在线。55 项测试通过，含两种封装、大小端、行填充、颜色通道、H.264 解码和显示限流。
独立测试查看器已确认图像进入主页面头部栏。最初输入近乎蓝灰色，使用者随后确认头部存在遮挡并已移开。移开后再次只读采样 5 秒，解出 61 帧、5 个关键帧，损坏帧为 0；实际图像可见地面、支架及周围物体，确认彩色光学画面恢复正常。验证截图已删除。解码器不显示标记为损坏的帧。

## 彩色图像启动

本机 ros2_humble 已安装 PyAV 17.1.0。其他电脑在可视化目录安装视频额外依赖：

```bash
python -m pip install -e '.[video]'
```

h1_2_mid360.yaml 保留原厂彩色视频来源；当前本机可编辑配置已改为下文 PC4 RealSense 话题。原厂视频配置如下：

```yaml
head_camera:
  video_topic: /frontvideostream
  video_height: 360
  rgb_hz: 5.0
  max_rgb_width: 640
```

开发电脑运行：

```bash
conda activate ros2_humble
cd /home/kaixiang/sariol_ws
source src/humanoid_bridge/setup_unitree.sh
cd src/humanoid_bridge/visualization
h1-2-viewer --config models/custom/h1_2/viewer.yaml --observe-client --serve --language zh
```

停止旧查看器后重启、打开新打印的网址；主页面下方左侧显示头部 RGB。当前可编辑配置读取 PC4 的标准 ROS 彩色图。图像按最多 5Hz、640 像素宽、JPEG 质量 80 发送，独立于模型和控制 Client。连接状态显示头部彩色／深度流的新鲜度。
仅 RGB 显示不需要虚构三维 TF 或内参；不旋转、镜像原始画面，也不把视频当作深度图。

标准 ROS Image 也可独立接入，设置 video_topic: null 和 rgb_topic: 实际话题。支持 rgb8/bgr8/rgba8/bgra8/mono8，按消息 step 处理行填充。JSONL 支持 head_rgb 事件（rgb: H×W×3 uint8 数据），Python API 为 dashboard.head_rgb(rgb)。两种 RGB 来源择一，避免互相覆盖。

## 接线调整前的调查记录

以下为 USB 改接前的历史调查，当前实测状态见文末 PC4 RealSense 采集。

使用者提供的 PC4 检查结果：realsense2_camera 未安装；未见相机相关进程；lsusb 没有 D435。不能据此在 PC4 安装驱动后就声称深度可用，也未向 PC4 上传脚本、改变 USB 接线或停止原厂采集。
进一步只读按 `/frontvideostream` 发布端的 DDS GUID 匹配接收包头，184 条匹配记录的源 IP 均为 192.168.123.161（PC1）。这确认了原厂视频的网络发布端，不单独证明相机 USB 的实际接线归属。PC1 的 22 端口返回 OpenSSH banner，用户确认没有 PC1 登录账号，未尝试密码或改变访问控制。原厂视频使用裸 DDS，ROS 节点名为 _CREATED_BY_BARE_DDS_APP_，不能通过节点名直接定位 USB 设备。
因此深度采集目前缺少可访问的采集主机／厂家公开深度输出。公开 Unitree videohub API 只查到 GetImageSample（1001），未找到可据以接入 H1-2 原始深度与内参的接口定义，不能把彩色截图接口当作深度接口。下文补充了官方仓库协作者提供的 H1／H1-2 相机 USB 接入开发机的路线；当前机器的接线尚未核对，无需先提供 PC1 密码，也不盲目停止控制／相机服务。

可向厂家提供以下问题描述（本项目不会自动发送）：

> H1-2 原装头部 D435：PC4 为 192.168.123.164，其 lsusb 未识别 D435；原厂 `/frontvideostream` 彩色流已接收，DDS 发布源为 PC1 192.168.123.161。我们无 PC1 登录账号，需要只读获取原始深度图、对应相机内参和光学坐标 TF，用于 ROS 2 Humble 可视化。请提供本型号的深度开发接口或原装相机接入开发计算单元的方法，并说明是否可保留原厂彩色视频服务。

下一步需要确认 D435 实际连接的计算单元。在识别到真实 D435 的计算单元上，使用 [RealSense 官方 ROS 2 驱动](https://github.com/realsenseai/realsense-ros) 提供校正深度 Image、对应 CameraInfo 与 TF，再通过 ROS 2 输送给开发电脑。若设备正被原厂服务占用，先确认现有服务用途，不能盲目抢占设备。

查看器已有 depth_topic + info_topic 接口，支持 16UC1 毫米／32FC1 米；深度图必须匹配 CameraInfo 的 frame、尺寸和校正内参。三维反投影通过真实 optical TF 接入 URDF，不把 camera_link 直接冒充光学坐标系。当前深度话题默认关闭，三维头部深度层不填充模拟数据。
配准、深度到 RGB 的对应、测量原点／光学轴以及时间同步须依据真实驱动输出核对，不能从 H.264 视频头获得。

## 无需登录 PC1 的接口复核

2026-10-29 再次核对官方文档和当前官方源码，仅查阅资料、发现话题和连接已知默认端口，没有启用、停止或重配任何机器人服务。

- [H1-2 基础服务](https://support.unitree.com/home/en/H1_developer/H1-2_Basic_Services_Interface) 列出 lowcmd/lowstate；本次未找到头部原始深度及内参的公开请求定义。
- unitree_sdk2 提交 d1abf001220d3e5f59e00a0c4aaa595e00089a4a、unitree_sdk2_python 提交 814556d15970dd2ecf1c9984e845ca02ab07e206：公开 VideoClient 为 GetImageSample；[相机例程](https://github.com/unitreerobotics/unitree_sdk2_python/blob/814556d15970dd2ecf1c9984e845ca02ab07e206/example/go2/front_camera/camera_opencv.py) 用 IMREAD_COLOR 解码，没有可据以接入原始深度的方法。
- [UnitreecameraSDK](https://github.com/unitreerobotics/UnitreecameraSDK) 提交 ecd2058ba3f033af11c2d2a0306b44cc8d819332 是 Go1 双目相机 SDK；getDepthFrame 例程启动本地相机捕获和双目计算，不是 H1-2 D435 的远程取深度客户端，未运行该例程。
- [TeleImager server.py](https://github.com/unitreerobotics/teleimager/blob/d90ab16d918324f94cf3fa70f2b8c54059012cc0/src/teleimager/server.py) 提交 d90ab16d918324f94cf3fa70f2b8c54059012cc0 的 RealSenseCamera 确有 enable_depth（默认 false）、get_depth_frame 和内参，但 _zmq_pub 仅发送 JPEG，所查 client.py 未提供深度接收接口。不能把采集支持误认为完整网络深度链路。
- 当前 DDS 域只读发现结果中，没有 sensor_msgs/Image、CompressedImage、CameraInfo 或名称含 depth 的话题。PC1 .161、PC4 .164 的官方 TeleImager 默认配置端口 TCP 60000 均拒绝连接；不排除非默认端口或私有接口。

因此，沿用现有网线、不登录 PC1、不改机器人服务直接订阅深度的现成入口仍未验证。没有找到公开入口，不等于固件绝不支持私有或尚未公开的接口。

## 官方仓库给出的 USB 采集路线

机器人充电期间继续查阅资料，找到 [xr_teleoperate #34 中协作者 silencht 的答复](https://github.com/unitreerobotics/xr_teleoperate/issues/34#issuecomment-2673199058)，发表于 2025-02-21。答复明确提及 H1 和 H1_2 的相机 USB 线从颈部引出，可插入开发计算单元（DCU）的 USB 接口；在该计算单元上通过 pyrealsense2 读取真实相机。

这提供了无需 PC1 账号的另一条路线：

```text
头部原装 D435 → USB → PC4 上的 RealSense ROS 2 驱动
                         → 深度 Image + CameraInfo + TF
                         → ROS 2 网络 → 开发电脑查看器
```

USB 引出线的存在及当前去向仍须在这台机器人上确认。该答复不是当前全部 H1-2 批次的接线图，也没有证明本机已经接通。若把原先供 PC1 使用的相机连接改到 PC4，可能同时失去现有 PC1 原厂 RGB 视频；不能承诺两路保留。只改变 USB 去向并不改变相机的机械安装外参。

答复中关于当时尚未发布图像流的描述是 2025 年的固件状态，不能用来否定本机已实测可用的 `/frontvideostream` 彩色视频。另一个 [G1 相机讨论 #106](https://github.com/unitreerobotics/unitree_sdk2_python/issues/106) 的成功脚本依赖开发机本地 USB 枚举，且作者注明测试型号为 G1 EDU-U4；不能据此推断给当前 PC4 安装 pyrealsense2 就能跨网络发现 D435。

恢复真机验证后的步骤是：先确认原装相机 USB 引出线及现有接法；确定采用 PC4 直采后，再接入并确认 PC4 能识别 D435；随后安装并配置 RealSense ROS 2 驱动，读取真实深度、内参与光学 TF，接入已有查看器接口。若要求保留全部原厂接线和服务，仍需厂家提供可用网络深度出口。

本次后续工作仅在线查阅资料、更新本文，未连接机器人、上传脚本、变更服务或 USB 接线；资料在内存中读取，没有新增下载副本或测试录制。

## USB 位置与双手 D435 接入

用户确认两只手部相机均为 RealSense D435。以下为待实施的接线方案，不表示已连接或已收到其图像。

[H1-2 官方使用手册](https://marketing.unitree.com/article/en/H1-2/User_Manual.html) 印刷页码 5 的 Interface Description 提供了[右侧接口板图](https://doc-cdn.unitree.com/static/2025/4/18/34aff4c8bcb448abadb51c065d281e28_2481x3509.png)：图中右侧中部四口 USB 组，上排①为 PC4 USB3.0、②为 PC4 USB3.0／DP；下排③④归 PC5。使用时对照实机标识，不能把 PC5 端口当成 PC4。该图没有标出头部相机线的具体插头；颈部引出线的依据是前述官方仓库协作者答复，尚未找到本机盖板内走线的公开实物定位图。

建议头部相机占用 PC4 的一个 USB 3.0 端口，两只手部 D435 经带独立供电的 USB 3.0 集线器接另一个 PC4 端口。头部是否改接单独决定，不必为了先接手部相机而改变原厂头部视频链路。D435 相机端为 USB-C 3.1 Gen 1，须使用支持 USB 3 数据传输的线缆；电脑端插头按 PC4／集线器实际接口选择。依据：[D435 规格](https://www.realsenseai.com/products/stereo-depth-camera-d435/)、[RealSense 多相机指南](https://dev.realsenseai.com/docs/multiple-depth-cameras-configuration/)。端口数量不代表独立 USB 控制器数量，三个相机的分辨率与帧率需按实际带宽配置。

两只 D435 应按真实序列号分别绑定 left_wrist、right_wrist，头部另用 head 名称；[RealSense ROS 2 驱动](https://dev.realsenseai.com/docs/ros2-wrapper/) 支持 serial_no、camera_namespace、camera_name。相机通过 USB 采集，与灵巧手的 CH340／ttyUSB0 串口控制独立；相机的颜色、深度、内参与 TF 也应各自命名，避免互相覆盖。手部相机的机械安装外参须依据实际安装核对后再接入机器人 TF。

宇树 [Device.md](https://github.com/unitreerobotics/xr_teleoperate/blob/main/Device.md) 的腕部 D405＋USB3 集线器接法可作电气拓扑参考，但其中支架、端口编号和安装图针对 G1，不能直接套用 H1-2；本项目不据此下载或安装 D405／G1 支架。

## PC4 RealSense 采集

2026-10-29 使用者将头部 USB 接到 PC4，SDK 实际识别到 D435i（序列号 337122072045，固件 5.15.1.55），USB 3 链路正常。安装 ROS Humble 的 realsense2_camera、realsense2_description 后，驱动日志显示 RealSense ROS／LibRealSense 均为 2.58.4。未更改相机固件。

PC4 在已安装驱动的环境运行，保持此终端开启：

```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=camera camera_name:=head \
  serial_no:=_337122072045 \
  enable_color:=true enable_depth:=true \
  rgb_camera.color_profile:=640x480x15 \
  depth_module.depth_profile:=640x480x15 \
  align_depth.enable:=true pointcloud.enable:=false
```

开发电脑跨机只读采样 8 秒，收到彩色、深度各 120 帧和内参 121 条。深度为 16UC1、毫米单位，尺寸 640×480；一次采样中 304643 个像素非零，中位有效深度 2.411 米。这些数值只描述当次场景，不作为固定阈值或标定值。

| 数据 | ROS 2 话题 |
| --- | --- |
| 彩色 | /camera/head/color/image_raw |
| 对齐到彩色的深度 | /camera/head/aligned_depth_to_color/image_raw |
| 对应内参 | /camera/head/aligned_depth_to_color/camera_info |
| 相机内部静态坐标 | /tf_static |

上述图像与内参均使用 head_color_optical_frame。实测内参 fx=606.1124、fy=605.6574、cx=324.5055、cy=243.0622，畸变系数均为零；查看器每次读取 CameraInfo，不把这些数值写死在代码里。相机内部 TF 使用驱动发布的 head_link → head_color_frame → head_color_optical_frame，保留真实深度到彩色的偏移和旋转。

本机配置的 camera_link → head_link 为同原点同轴的名义安装假设，挂接在用户可编辑 URDF 上，不冒充测量标定。完整链为 torso_link → camera_link → head_link → head_color_frame → head_color_optical_frame。若实物与模型仍有偏差，应修改该安装桥接的配置或重新标定，不改像素、不直接翻转深度点云，也不覆盖驱动内部外参。

可编辑配置关闭 video_topic，启用标准 rgb_topic、depth_topic、info_topic，避免旧原厂 RGB 和新 RGB 相互覆盖。彩色、深度分别以 rgb_hz、depth_hz 限制显示频率，默认 5Hz；本机 stride=8，640×480 深度每次最多反投影 4800 点。接收端只保留最新图像，PC4 不额外发布稠密点云。上述启动仅采集相机，查看器仍不发送机器人运动命令。

只读适配器联调 6 秒，处理身体／IMU 各 55 批、彩色／深度各 54 批，深度抽样得到 4755 点，光学坐标到躯干的 TF 查询成功，无错误或未连接的传感器图层。该次未收到雷达或手部数据，未启动它们。联调关闭 Rerun 渲染输出，因此不据此宣称浏览器帧率或视觉标定已验证；完整离线测试 56 项通过，包含深度反投影与图像输出限频。

## 彩色深度点云

本机可编辑配置新增：

```yaml
head_camera:
  colorize_depth: true
  rgbd_tolerance_ms: 0.0
```

必须使用已对齐到彩色的深度话题，RGB 与深度的尺寸和光学 frame_id 必须相同。实际相机采样确认可以匹配相同时间戳，因此使用 0 毫秒容差，未开启相机上的其他同步选项。RgbdPairer 独立于 ROS 与渲染，只做有界的消息关联；每路最多保留 4 帧，收到任一路后尝试产生最新完整配对，每帧只用一次，丢弃晚到的已消费帧。未配对的彩色图仍可作二维显示，深度三维层等待新配对，不沿用旧颜色。

配对完成后用深度和 CameraInfo 反投影三维点，再以同一像素索引抽取 RGB，深度为零、非有限值或超过距离限制的像素及其颜色一起过滤。位置计算仍采用米制深度，不用颜色估计距离。网页输出仍受 depth_hz／rgb_hz 限频和 stride 抽样约束，不额外发送全部高密度彩色点云。关闭 colorize_depth 可独立恢复原来的单色深度层。rgbd_tolerance_ms 可为其他数据源设置小幅时间容差，但不能解决未做空间配准的问题。

离线测试增至 62 项，覆盖两路消息的到达顺序、精确／容差配对、不同尺寸与坐标拒配、单路断流缓存上限、晚到旧帧、使用配对颜色而非最新颜色，以及无效深度像素的对应颜色过滤。

真机只读联调 7 秒匹配 105 对 RGB-D，配对时间戳差为零；最终点云 4777 点，颜色数组为 4777×3 uint8，含 709 种 RGB 值，无配对或 TF 错误。联调未开启额外浏览器或保存录制，也未启动雷达、手部或运动控制。本次修改需重启电脑端查看器，PC4 相机启动命令不变。
