# 真机 MID360 融合显示

PC4 已部署驱动；使用者实测 /livox/lidar 为 PointCloud2，frame_id=livox_frame，单帧约 2 万点。
本机只读订阅 5 秒收到 50 帧点云、约 2500 帧身体状态；这次采样点云约为 10 Hz。
可视化不需要在 PC4 增加脚本，也不改身体／手部控制。

## 启动

PC4 保持原雷达驱动运行。在开发电脑执行：

```bash
conda activate ros2_humble
cd /home/kaixiang/sariol_ws
source src/humanoid_bridge/setup_unitree.sh
cd src/humanoid_bridge/visualization
h1-2-viewer --config models/custom/h1_2/viewer.yaml --observe-client --serve --language zh
```

更新代码后先 Ctrl+C 停止旧查看器、关闭旧网页，再执行新命令并打开新地址；旧进程不会自动加载修改。
原控制 Client、PC4 驱动继续独立运行。可使用 --language en 对照排查中文显示问题。

## 坐标连接

### 身体 IMU 姿态

本机可编辑配置和 MID360 配置已启用以下设置：

```yaml
attitude:
  enabled: true
  imu_frame: imu_link
spatial:
  world_frame: imu_world
```

查看器复用 `/lowstate` 的 `unitree_hg/msg/LowState.imu_state`，不增加控制节点、不订阅雷达 IMU 来冒充身体姿态。Unitree 四元数按 wxyz 读取，在独立的 attitude.py 转为可视化接口的 xyzw。
2026-10-29 的只读采样在 3 秒内收到 1501 条身体消息，四元数约 `[0.06749, 0.01651, 0.00800, -0.99755]`，消息 RPY 约 `[-0.79°, 1.95°, -172.27°]`；该瞬时读数只验证数据可用和解码方向，不构成实物外参标定。

官方 URDF 中 imu_link 固定在 torso_link，模型根为 pelvis。因此计算：

`R_world_pelvis = R_world_imu × transpose(R_pelvis_imu)`

其中 R_pelvis_imu 使用当前关节反馈和用户正在使用的 URDF，包含腰关节和 IMU 安装角度。若实际 IMU 对应其他机身坐标系，应修改 imu_frame 或可编辑 URDF；当前采用官方安装基线。
主场景使用独立的世界参考原点，模型和雷达一起随姿态旋转；仍以骨盆为观察原点会把整机姿态抵消掉。Debug → IMU 显示读数、是否应用；连接状态显示 IMU 新鲜度。
IMU 页同时显示横滚／俯仰曲线、独立航向曲线和 xyzw 四元数曲线，下方保留状态说明。姿态角单位为度，四元数无量纲；沿用 runtime.debug_hz（默认 2Hz）限流，不改变模型姿态更新频率。RPY 保留设备的角度回绕，跨越 ±180° 时可能出现跳变；JSONL 没提供 RPY 时由四元数计算。英文模拟数据录制已检查该页布局和图例，临时录制与截图验证后清理。

imu_world 只是 IMU 姿态参考：平移设为显示零点，不代表定位、脚底地面或吊挂高度；yaw 沿用设备参考，不等于地图北向。没有积分加速度估算位置，也没有强行把某个平面转成地面。
外部 robot_pose 或根坐标 TF 接入后优先于 IMU，IMU 不再覆盖根变换。接入地图定位时相应配置 spatial.world_frame；不要把 imu_world 当成已标定地图。
`attitude.enabled: false` 可关闭该功能；通用配置默认关闭。模拟演示保持原有模拟位姿，不使用真机 IMU。

JSONL 同样支持独立事件 `{"schema_version":1,"type":"attitude","quaternion_xyzw":[0,0,0,1],"rpy":[0,0,0]}`，RPY 是可选的诊断弧度值，旋转以四元数为准。

### 雷达安装基线

本机可编辑模型 viewer.yaml 和可分发的 h1_2_viewer/config/h1_2_mid360.yaml 已加入：

```yaml
spatial:
  static_transforms:
  - parent: lidar_link
    child: livox_frame
    position: [0.0, 0.0, 0.0]
    quaternion_xyzw: [1.0, 0.0, 0.0, 0.0]  # Rx(pi), checked against this robot's IMU
```

这是查看器内部的命名静态 TF，不向 ROS /tf_static 发布，避免改动控制系统的坐标树。
URDF 的 torso_link → lidar_link 提供官方安装基线：平移 [0.04874, 0, 0.67980] 米，RPY [0, 0.24015730507441985, 0] 弧度。
实际 torso_joint 从 /lowstate 更新，所以雷达随真实腰部关节变化。

原先将零外参驱动输出的原始雷达坐标与官方 lidar_link 设为重合，经真机 IMU 检查不成立。现通过这条独立 TF 加入绕 X 半周的安装旋转，未修改原始点云、官方 URDF 或 PC4 驱动；驱动 JSON 的 extrinsic_parameter 继续保持零。
这解决已观测到的竖直方向颠倒，不代表完成全部外参标定。航向仍待实物标识确认，平移暂沿用官方安装原点，尚未测量原始点云原点的附加偏移。若地面或墙面有系统性偏差，编辑上面的 position / quaternion_xyzw 校准；不要重复叠加 torso_link → lidar_link 的安装变换。
同时检查实际安装方向和驱动外参，不能仅靠话题 frame_id 名称证明物理对齐。
验证已用真实采样显示机器人与点云，场景没有 TF 断开报告；未进行地图定位、运动去畸变或时间插值。
2026-10-29 用户贴出的 PC4 当前 JSON 已核对：雷达 192.168.124.20，主机 cmd/push/point/imu 接收均为 192.168.124.164，extrinsic_parameter 的 roll/pitch/yaw/x/y/z 六项全零，未在驱动中叠加安装变换。
实物完整朝向仍待确认；不能根据点云斜面单独认定它是地面或证明外参正确。
修正安装 TF 后只读运行约 5 秒，显示批次接收 46 组身体／IMU、45 帧点云，另采样 1000 条雷达 IMU，无错误或断开的 TF；雷达加速度经整条世界坐标链换算后，归一化方向约 `[-0.00874, -0.00078, 0.99996]`，与世界 +Z 的误差约 0.50°。49 项测试通过，完整官方 URDF 的 SHA256 不变。显示批次是抽取后的最新样本，不代表原始话题频率；临时录制通过 Rerun 校验，并在独立测试查看器中检查了模型与点云显示，验证后清理录制和截图。

## 2026-10-29 雷达方向只读复核

本节保留修正前的只读检查证据；修正后的生效配置和复测见上节。

- 宇树官方公开 [H1-2 URDF](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/h1_2_description/h1_2.urdf) 中，torso_link → lidar_link 为 xyz `[0.04874, 0, 0.67980]` 米、rpy `[0, 0.24015730507441985, 0]` 弧度，约绕 Y +13.76°，没有 180° 安装翻转。当前 upstream 和 custom 的完整 URDF SHA256 都与锁定清单的 `504b0cc9ea5311044876f4be4e3148fbc285aa1439e33b19d93c357a7468066c` 一致，未被改反。
- 该工程坐标系不自动证明它等于 MID360 的原始点云坐标。修正前配置将 lidar_link → livox_frame 设为单位变换，这是当时未经验证的假设。
- [Livox 官方手册](https://terra-1-g.djicdn.com/851d20f7b9f64838a34cd02351370894/Livox/Livox_Mid-360_User_Manual_EN.pdf) PDF 第 14 页给出点云坐标，第 17 页明确雷达 IMU 与点云的坐标轴方向相同。
- 已核对部署的固定版本驱动：`src/comm/lidar_imu_data_queue.cpp` 和 `src/lddc.cpp` 将 IMU acc_x/y/z 原样传递，点云 x/y/z 也未在发布层交换或取反；PC4 用户贴出的外参六项全零。本机 pointcloud.py 按 x/y/z 原样读取，SensorScene 用命名坐标系绘制，不再手动乘一次安装矩阵。

在机器人保持近静止时，3 秒只读采样得到 1506 条身体消息、600 条雷达 IMU。为消除两种驱动的加速度量纲差异，仅比较单位方向：

| 项目 | 单位向量／角度 |
|---|---|
| 身体四元数预计的向上方向，表达在 torso/imu 坐标 | `[-0.03417, -0.01394, 0.99932]` |
| 身体加速度均值归一化 | `[-0.03823, -0.01275, 0.99919]` |
| 雷达原始加速度均值归一化 | `[-0.26300, 0.00929, -0.96475]` |
| 修正前 TF 换至 torso 后，与预计向上的夹角 | `149.03°` |
| 仅离线试算 lidar_link → livox_frame 绕 X 半周的候选关系 | `0.54°` |
| 仅离线试算绕 Y 半周的候选关系 | `30.99°` |

结论：原先单位变换与静止 IMU 的竖直方向明显矛盾，问题位于原始 Livox 坐标与 URDF lidar_link 的连接，而非用户把官方 URDF 改反。用户随后授权继续，已在单独的传感器 TF 应用有数据支持的绕 X 半周关系，并完成上节复测；不能仅凭这些数据确认航向、安装平移或全部实物外参。官方模型基线和原始点云保留。

## 原装安装的官方资料复核

已直接读取宇树官方 [About H1-2](https://support.unitree.com/home/en/H1_developer/About_H1-2) 正文、[结构图](https://doc-cdn.unitree.com/static/2024/9/4/9e873852c7e24cb7bbae7f19868483f8_2730x1784.png)、[关节坐标示意图](https://doc-cdn.unitree.com/static/2024/9/5/928eea81692d486b82ae056f2a7865f4_450x1190.jpg)，以及 [SLAM Service](https://support.unitree.com/home/en/H1_developer/SLAM%20Service)、[SLAM Example](https://support.unitree.com/home/en/H1_developer/SLAM%20Example)。H1-2 页面确认头部 MID360，并列出当前雷达 IP 192.168.124.20（部分旧版为 192.168.123.120）。

官方原装的名义安装参数采用上节 URDF：相对 torso_link 前移 48.74mm、侧移 0、高 679.80mm，工程坐标 pitch +13.76°、yaw 0。当前两个 MID360 配置已保持这些值，没有另加航向角或安装平移。用户已确认原装固定安装，不要求用户重新测量这些官方名义尺寸。

合并传感器轴向修正后，当前采用的 `torso_from_livox` 名义变换为：

```text
position_m = [0.04874, 0, 0.67980]
rpy_rad    = [pi, 0.24015730507441985, 0]
R = [ 0.97130057,  0, -0.23785542
      0,          -1,  0
     -0.23785542,  0, -0.97130057 ]
```

约定为 `p_torso = R * p_livox + position_m`。该关系下原始雷达 +X 朝机器人前下方，+Y 朝机器人右侧，+Z 朝后下方；这里描述的是当前采用的坐标关系，不能把它误引为官网直接公布的原始传感器完整外参。腰部转动时还需组合实时 pelvis → torso_link 变换。

两份生效配置的矩阵一致，旋转行列式为 +1，完整模型 SHA256 不变。平移／工程 yaw 来自官方 URDF，绕 X 半周来自本机 IMU 验证，两类来源分别保留。

公开页面有结构与关节坐标图，但没有找到把 `lidar_link` 与 Livox 原始点云原点、X 轴逐项对应的完整六自由度表；官方 SLAM 示例链接的 `unitree_slam_h1` 分支（复核提交 1e49bfa4dca2c992a566ee7d1d8480d4102149b8）也未公开机器端完整外参配置。因此当前结论是“官方原装名义值已对照，附加旋转已验证竖直方向”，不声称航向和平移已经独立实测标定。若需要进一步的厂家原始坐标依据，应读取 PC4 出厂 SLAM 外参文件或取得厂家安装图，不根据模糊照片或手册尺寸猜测补偿。
Livox 手册的 `[11.0, 23.29, -44.12]mm` 是雷达内部 IMU 芯片相对点云原点的偏移，不是机器人到雷达的安装平移，不能直接加到当前点云 TF。

## 显示负担

默认显示设置：模型 10 Hz，曲线 2 Hz，点云最多 5 Hz / 4000 点 / 4cm 体素；完整原始消息不被修改。
接收线程只保留每流最新数据，渲染／网络阻塞时不会阻止 DDS 接收继续替换最新样本。
控制目标和实际状态分离；显示限流不影响 Client 的控制频率。

关键修正：官方 URDF 加载器同时载入 collision 几何，原先仅简化 visual，仍留下了高面数碰撞副本。
现在在临时显示 URDF 中移除 collision；原始模型和碰撞资源不变。需要调试碰撞几何时可设置 model.show_collisions: true，但会显著增加负担。
已取消默认网格减面，使用原始 visual 网格（约 79 万面）。此前每部件 500 面的简化让肘、肩、髋部发生明显尺寸损失，不能用于忠实显示。
保留去除重复 collision、数据降频和点数限制，不再用改变机器人形状换性能。
关节显示变换使用 0.001rad 的更新阈值，消除静止传感器微抖的重复日志；原始反馈和曲线不应用该阈值。

曲线由每关节一条日志改为每组一个批次，不再重复写固定坐标字段和未变化面板。
默认不记录第二份 lidar_raw，Rerun 服务保留 64MiB 并优先向新连接发送最新数据。
历史减面版本的录制大小不能代表当前原始外观版本的负担；未对恢复原始网格后的浏览器 FPS 作保证。
英文浏览器页已实际加载真机融合场景；中文无窗口诊断器缺少 CJK 字体，因此该诊断器的场景检查使用英文，不代表已验证所有浏览器的中文字体行为。

可在 YAML 的 runtime 中修改 debug_hz、lidar_hz、lidar_max_points、lidar_voxel_size、log_raw_lidar、server_memory_limit。
本次没有调整机器人网络、PC4 软件或控制程序。只读测试实例、录制和截图在验证后清理。
