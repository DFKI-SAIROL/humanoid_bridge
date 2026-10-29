# 统一场景与远程部署

Observe 页上方为机器人主场景，下方从左到右排列头部、左手、右手 RGB。头部二维深度图位于 Debug 页。任务概览已删除，诊断信息集中在 Debug 页。
左侧 Blueprint 默认展开，再展开主视图的实体树，点击眼睛分别隐藏/显示：
- 机器人模型：URDF 几何所在分支（robot 下的 URDF 链接树）。
- 雷达：`robot/sensors/lidar`。
- 头部深度点云：`robot/sensors/head_depth`。

也可以保存自己的 .rbl，启动时用 --blueprint 加载。隐藏模型时仅隐藏几何分支，不要关闭整个 robot（它还包含传感器）。

## 坐标与数据

沿用锁定版本的官方模型，不覆盖用户模型：torso_link -> camera_link 和 lidar_link 固定安装外参由 URDF 提供。
来源：https://github.com/unitreerobotics/unitree_ros/blob/master/robots/h1_2_description/h1_2.urdf
实现参考 Rerun 命名坐标系：https://rerun.io/docs/concepts/logging-and-ingestion/transforms

雷达 header.frame_id 必须对应 lidar_link 或经真实 TF 接通的坐标；禁止随意把驱动 frame 改名来伪造对齐。
头部 camera_link 不等于深度光学坐标：必须提供真实 optical-frame TF；光学轴 X右、Y下、Z前。
模型的安装外参是基线，真实设备型号/安装方向和驱动坐标需实机核验。

Python API：
```
dashboard.head_depth(depth_m, K_3x3, 'camera_depth_optical_frame', rgb=registered_rgb)
```
深度为校正后的 HxW 米值；可选 RGB 必须已配准到深度。0/NaN/Inf 和超过 max_depth 的像素不生成点。
默认 stride=4、max_depth=8m。没有 TF 时等待，TF 到达后显示；不做配准优化、SLAM、时间插值或多传感器时间同步。
点云随最新 TF 重投影；高速运动精确融合需后续按传感器时间戳查询 TF。

ROS YAML（话题需换成驱动实际名称）：
```yaml
head_camera:
  depth_topic: /camera/depth/image_rect_raw
  info_topic: /camera/depth/camera_info
  stride: 4
  max_depth: 8.0
spatial:
  ros:
    tf_topic: /tf
    tf_static_topic: /tf_static
```
仅订阅，不启动驱动。支持 16UC1 毫米和 32FC1 米，处理大小端及行填充。
CameraInfo 必须与图像同 frame/分辨率且对应零畸变校正图，内参固定最新值。
实时 ROS 接口支持独立 RGB Image 和深度，原厂 H.264 视频另有只读适配，详见 [头部相机](HEAD_CAMERA.md)；未启动 RGB 驱动或实现 RGB-D 同步器。
JSONL 新事件 type=head_depth，字段 depth、intrinsics、frame_id，可选 rgb、stride、max_depth、elapsed。
现有独立 ROS 导出脚本可导出 head_rgb，尚不导出头部深度；头部深度 ROS 接入目前需使用同环境实时模式。

## 远程运行

在有数据的一侧启动无桌面服务（demo 不连接机器人）：
```bash
h1-2-viewer --demo --serve --grpc-port 9876 --web-port 9090
```
演示结束保持服务直到 Ctrl+C。真实数据去掉 --demo 并加载真实配置，不在本次操作中执行。
服务默认保留最近 64MiB 数据；浏览器渲染需要查看电脑有 WebGL 支持。

优先通过 SSH 转发在本机访问：
```bash
ssh -N -L 9090:localhost:9090 -L 9876:localhost:9876 user@robot-or-server
```
浏览器打开 http://localhost:9090；若未自动连接，在 Viewer 打开
`rerun+http://127.0.0.1:9876/proxy`。
服务没有应用层登录认证；在受信网络/SSH/VPN 内使用，端口不要直接暴露公网。
不要把机器人 DDS 多播跨公网转发；ROS 采集与渲染输送分离。
另一种方式：本地电脑运行 Rerun 接收端，采集进程使用
`h1-2-viewer --connect rerun+http://VIEWER_HOST:9876/proxy`。

待和用户确定：同局域网/跨互联网、机器人端算力、浏览器或原生客户端、带宽和帧率、是否需要多人查看。
