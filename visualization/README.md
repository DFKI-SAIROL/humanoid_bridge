# 独立 H1-2 Rerun 查看器

本目录可以单独复制、安装和运行，不依赖 `whole_body_controller` 的 Python 导入或 `robot_bridge` 的 CMake 安装。
离线模式不需要 ROS；实时模式才加载 ROS 适配器。仅接收数据，不导入控制器或发送命令。全身控制 Client 与可视化独立运行，见 [并行观察说明](docs/CLIENT_OBSERVER.md)。

## 浏览器启动

在自己的电脑运行程序，由这台电脑提供可视化网页，无需公共服务器。
首次从 Git 源码安装，在 Python ≥3.11 的环境中执行：

```bash
cd humanoid_bridge/visualization
python -m pip install -e .
h1-2-models fetch
h1-2-viewer --demo --serve
```

终端会打印可直接打开的完整地址，例如：

```text
可视化地址：http://localhost:9090/?url=rerun%2Bhttp%3A%2F%2Flocalhost%3A9876%2Fproxy
```

在运行程序的电脑上用浏览器打开此地址。程序保持运行，按 `Ctrl+C` 停止服务。
换电脑后重新安装依赖和获取模型即可，不需要修改源码中的绝对路径。
端口被占用时使用 `--web-port 9092 --grpc-port 9877`；打印的地址会对应更新。
不加 `--serve` 仍打开原生 Rerun 窗口。图层开关及头部相机数据要求见 [统一场景说明](docs/UNIFIED_SCENE.md)。

接入机器人时，在已配置 ROS 和消息依赖的环境运行 `h1-2-viewer --config YOUR_CONFIG.yaml --serve`。
使用者在配置文件中设置自己的模型、话题、传感器坐标及标定；默认仅订阅，不启动机器人或驱动。控制动作由独立 Client 负责；使用 `--observe-client` 可只读显示它发出的目标值，详见 [并行观察说明](docs/CLIENT_OBSERVER.md)。
系统 ROS Humble 的 Python 3.10 与查看器需分环境运行，具体见下文跨环境说明；头部深度的独立导出适配尚未实现。

## 真机雷达与性能

本机 MID360 已接入官方 URDF 安装基线。使用原来的可编辑模型配置即可，无需在 PC4 添加脚本。
原始 livox_frame 与模型 lidar_link 之间已根据本机 IMU 对照补入绕 X 轴半周的安装旋转，修正竖直方向颠倒；航向和安装平移仍待标定。
真机配置同时读取身体 IMU，结合腰关节反馈换算整机姿态；Debug 页可查看 IMU。当前只提供姿态参考，未测得世界位置、吊挂高度，也未完成实物雷达外参标定。
详细的 TF 假设、可编辑外参和实测结果见 [真机 MID360 融合显示](docs/MID360_LIVE.md)。

## 头部相机

本机头部相机实际为 D435i，已通过 USB 3 接入 PC4，并由 RealSense ROS 2 驱动发布 640×480、15Hz 彩色、对齐深度和内参。可编辑配置 models/custom/h1_2/viewer.yaml 已切换到这些真实话题，并订阅相机静态 TF；网页彩色与深度最多各显示 5Hz，三维深度按像素间隔 8 抽样。
本机配置同时启用彩色深度点云：仅配对同时间戳、同光学坐标和同尺寸的 RGB／对齐深度，按相同像素取真实颜色。配对缓存每路最多 4 帧，缺失或未配对的图像不用于点云上色；可独立关闭 head_camera.colorize_depth 恢复单色深度层。
相机内部外参与光学轴采用驱动 TF，URDF camera_link 到驱动 head_link 暂按原装安装位置同原点同轴连接，尚未做实物外参标定。原厂 `/frontvideostream` 方式仍可通过 h1_2_mid360.yaml 选择，使用该方式需安装视频额外依赖 `python -m pip install -e '.[video]'`。PC4 启动命令、接口和安装假设详见 [头部相机说明](docs/HEAD_CAMERA.md)。

## 显示性能与语言

默认使用原始外观网格，`model.mesh_face_limit: 0`。此前的减面会使部分肘、肩、髋部尺寸损失，已取消默认启用。
碰撞几何默认不载入查看器，防止与外观模型重复渲染；原始 URDF 和碰撞资源保留。
关节坐标和运动学不变；只近似外观细节，不修改用户 URDF、原始网格或碰撞资源。
仅在显式设置非零 mesh_face_limit 时生成临时显示网格；默认直接使用原始资源。
`--full-detail` 可强制使用原始外观；默认 `model.mesh_face_limit: 0` 也采用原始模型。非零值仅供显式实验，不保证保持所有部件形状。
首次更新依赖执行 `python -m pip install -e .`；本机环境已安装新增依赖。

```bash
h1-2-viewer --demo --serve --language zh  # 中文页面和标签
h1-2-viewer --demo --serve --language en  # 英文页面和标签
```

语言也可通过 YAML 的 `layout.language: zh` 配置。该选项覆盖本项目页面、图像标签、关节分组和连接状态；
Rerun 原生菜单、实体路径、技术诊断详情仍使用原有名称，不表示整个 Rerun 已汉化。
重复图像、未变化的关节变换和固定坐标引用不会重复发送；统一场景只绘制一份目标与轨迹。

本机同一 420 帧演示：录制约 46.3 → 26.2 MB，数据生成约 2.9 → 1.9 秒（不含模型初始化）。
性能数字只衡量数据负担，不能代替浏览器帧率测试。
若浏览器仍明显卡顿，需要核查浏览器图形加速和 GPU 后端；原生模式可用于对照排查。

## 完整模拟任务演示

```bash
conda activate ros2_humble
cd humanoid_bridge/visualization
h1-2-viewer --config models/custom/h1_2/viewer.yaml --demo
```

默认实时模型 10 Hz、关节调试数据约 2 Hz、连接状态约 1 Hz；传感器仅在新数据或待补齐 TF 到达时发送，减少浏览器负担。

默认演示时间轴为 21 秒、420 个采样帧，依次经过：导航到桌前 → 停稳 → 观察目标 → 抓取规划预览 → 接近并闭手 → 抬起 → 确认结果。
录制数据会快速生成，Rerun 用底部 `elapsed` 时间条播放／暂停和循环回看；这不是机器人实时执行。

| 区域 | 内容 |
|---|---|
| Observe 上方 | 机器人主场景，叠加可选择显示的雷达、头部深度点云、环境及轨迹 |
| Observe 下方 | 从左到右：头部、左手、右手三路 RGB 图像 |
| Debug 页 | 连接状态、关节曲线和数值、TF、传感器对齐 |

全部场景、关节运动、相机外参／内参、图像、深度和检测轮廓均为脚本模拟。它不是物理仿真，也没有实际 locomotion、SLAM、视觉识别或抓取算法。
主视图跟随 URDF 根节点；环境通过显式机器人位姿和 TF 显示世界位置。模拟定位不能说明真实机器人能完成任务。
手指模型尚未标定，闭手阶段展示的是模拟通道变化，未声称真实手指已完成闭合。
任务进度面板已移除；任务数据接口保留给上层集成，演示不再逐帧生成任务面板。
不使用 `--demo` 时，不会自动生成相机数据。

本项目是 editable 安装，修改后重新运行上述命令即可，不需重新编译 bridge。
默认展开左侧 Blueprint 图层栏，点击场景实体旁的眼睛控制显示；雷达为 `robot/sensors/lidar`，头部深度点云为 `robot/sensors/head_depth`。头部二维深度图放在 Debug 页。手部 RGB 沿用 camera 接口，真实 ROS 图像驱动接入仍需配置适配。
若已有旧 Rerun 窗口仍显示旧布局，关闭旧窗口再运行，或在 Viewer 中恢复默认 Blueprint。

## 文件在哪里、改什么

```text
visualization/
├── pyproject.toml                   # 独立 Python 包及 h1-2-viewer / h1-2-models 命令
├── environment.offline.yaml         # 不含 ROS 的环境配方
├── environment.ros2.yaml            # 可选 RoboStack 实时环境配方
├── requirements.tested-linux-py314.txt  # 本机验证时的 Python 依赖快照
├── h1_2_viewer/
│   ├── config/default.yaml          # 话题、关节索引、模型显示、布局默认配置
│   ├── config/model.lock.json       # 官方模型提交与逐文件 SHA-256
│   ├── configuration.py             # 配置读取；相对路径以 YAML 所在目录为基准
│   ├── assets.py                    # 下载、校验、生成可编辑模型副本
│   ├── model.py                     # 官方 Rerun UrdfTree 模型适配
│   ├── telemetry.py                 # Unitree / JointState → 统一关节数据
│   ├── pointcloud.py                # 点云解析，无 ROS 导入
│   ├── ros2_source.py               # 可选 ROS 只读订阅
│   ├── jsonl_source.py              # 跨 Python 环境的单向数据输入
│   ├── demo.py                      # 独立合成数据源
│   ├── renderer.py                  # 曲线、模型更新、状态显示
│   ├── layout.py                    # Blueprint 布局，可独立替换 .rbl
│   └── extensions.py                # 地图、路径、控制状态显示接口
├── tools/ros2_export.py              # Python 3.10+ ROS 采集端，不导入 Rerun
├── models/upstream/                 # 本机官方基线，自动校验，不应修改；Git 忽略
├── models/custom/h1_2/              # 本机已生成的可编辑、自包含模型；Git 忽略
│   ├── viewer.yaml                 # 此副本的配置，全部使用相对路径
│   ├── h1_2_description/
│   │   ├── h1_2.urdf               # 当前显示使用的模型，日常可改
│   │   ├── h1_2.xml                # MuJoCo 模型，日常可改
│   │   ├── h1_2_handless.*          # 其他官方变体
│   │   └── meshes/                 # STL 等网格，随副本一起迁移
│   ├── LICENSE
│   └── SOURCE.json
└── tools_export_bundle.py            # 生成包含可编辑模型的迁移包
```

旧入口 `python -m whole_body_controller.dashboard` 仅转发到本包，保留兼容性。
控制器本身仍在原来的 `whole_body_controller/h1_2_controller.py`，没有移入查看器。
最初用于下载的临时 Git 副本已清理；官方来源、提交和哈希保留在 `config/model.lock.json`，许可证随模型目录保留。

## 另一台电脑：只做可视化

需要 Python ≥3.11；Rerun SDK 固定为 0.40.0。该版本不能直接安装到 Python 3.10。
本机验证环境为 Linux x86_64 / Python 3.14。

```bash
# 在复制出来的 visualization 目录（或迁移包根目录）中
conda env create -f environment.offline.yaml
conda activate h1_2_viewer
python -m pip install -e .

# 如果迁移包已经包含 models/custom/h1_2，下面三条不需要联网下载模型。
h1-2-viewer --config models/custom/h1_2/viewer.yaml --check
h1-2-viewer --config models/custom/h1_2/viewer.yaml --demo
```

环境 YAML 是可重新求解的跨机器配方，**不是全平台字节级锁文件**。
同类 Linux/Python 3.14 环境可参考 `requirements.tested-linux-py314.txt`；其他平台以兼容版本重新求解。
不应把本机 Conda 的绝对 `prefix`、`build/install/log` 复制到另一台机器。

## 纯源码安装：获取官方模型

如果只有 Git 源码，没有迁移包中的模型：

```bash
python -m pip install -e .
h1-2-models fetch
h1-2-viewer --check
h1-2-viewer --demo
```

`fetch` 下载 `model.lock.json` 固定的提交，逐文件检查 SHA-256。
默认缓存是 `$XDG_CACHE_HOME/h1_2_viewer/models/<commit>`，未设置时为 `~/.cache/...`。
它不会自动跟随官方分支更新，也不会覆盖已有且不匹配的模型。
模型下载需要 Git 和网络；查看器运行不会隐式下载。

本机还有目录内基线 `models/upstream`，因此可以离线校验和创建新副本：

```bash
h1-2-models verify models/upstream
h1-2-models fork --source models/upstream --dest models/custom/experiment_01
h1-2-viewer --config models/custom/experiment_01/viewer.yaml --demo
```

只存在默认缓存时，省略 `--source models/upstream` 即可。

## 修改 URDF、XML、mesh 或关节映射

日常编辑 `models/custom/h1_2/h1_2_description/`，不要改 `models/upstream` 或用户缓存。
每个副本自带 mesh、许可证、来源信息及相对路径配置，可以整目录搬走。

- **Rerun 模型**读取 `viewer.yaml` 指定的 `.urdf`。修改后重新打开查看器，不需要重新编译 bridge。
- **MuJoCo**读取 `.xml`；它与 URDF 是两份模型，修改 XML 不会自动改变 Rerun 中的 URDF，反过来也一样。
- 将模型切换到 `h1_2_handless.urdf`，只改 `viewer.yaml` 的 `model.urdf`。
- 改电机顺序、模型关节名、方向、零位或曲线分组，配置 `joints`；不改控制器或 renderer。
- 改视角、背景、曲线默认分组和面板比例，配置 `layout`；也可保存自己的 `.rbl`，运行时传 `--blueprint my_layout.rbl`。
- `model.display_tint: null` 使用原始模型材质；RGBA 数组仅改变显示颜色，不会改写源文件。
- URDF `package://` 路径可用 `model.package_paths` 显式映射，不会把包名偷偷丢弃。
- Xacro 需要先生成普通 URDF；MuJoCo XML 不是 URDF，不能直接交给查看器。

示例局部配置，会继承未指定的默认值：

```yaml
schema_version: 1
model:
  urdf: h1_2_description/h1_2.urdf
  package_paths:
    my_hand_description: resources/my_hand_description
  display_tint: [0.52, 0.60, 0.70, 1.0]
source:
  type: unitree
  state_topic: /lowstate
  lidar_topic: /livox/lidar
  hand_topic: /inspire/state
runtime:
  hz: 20
  stale_after: 2
layout:
  default_joint_group: left_arm
```

`joints` 如需修改，请从 `h1_2_viewer/config/default.yaml` 复制完整列表后编辑；它是整体替换，不按位置合并。
每项包含 `name`、`motor_index`、`group`、`scale`、`offset`。
Unitree 输入：`q_model = q_motor * scale + offset`，`dq_model = dq_motor * scale`；力矩曲线保留原始电机估计值 Nm。
标准 JointState 输入已经是 URDF 坐标，不再次应用硬件 scale/offset。缺少速度或力矩时显示数据空缺，不伪造零值。
配置错误、缺失 joint/mesh、重复映射会在 `--check` 阶段明确报错。

## 实时数据：两种 Python 环境都可接

### 本机已有兼容 ROS 的 Python ≥3.11 环境

本机 `ros2_humble` 是 RoboStack / Python 3.14，可以直接运行。
先激活环境并 source **该机器**构建的 Unitree 消息工作空间；网络配置也使用该机器的接口。
查看器不读取 `setup_unitree.sh`，不把 `enp0s31f6` 固定在程序里。

```bash
h1-2-viewer --config models/custom/h1_2/viewer.yaml
```

默认输入：`/lowstate` (unitree_hg/LowState)、`/inspire/state` (unitree_go/MotorStates)、`/livox/lidar` (PointCloud2)。
只有使用 Unitree 输入时才需要相应消息包，不必运行机器人控制 bridge。

其他仿真器或状态发布节点可使用标准消息，避免依赖 Unitree 消息：

```bash
h1-2-viewer --config models/custom/h1_2/viewer.yaml \
  --source joint_states --state-topic /joint_states
```

这会默认关闭 Inspire 订阅；位置按 `JointState.name` 匹配，不依赖数组顺序。

### 系统 ROS Humble / Python 3.10

不要把 Rerun 0.40 强行安装到系统 ROS Python。采集端和显示端用单向 JSONL 管道分开：

```bash
# ROS 环境：source /opt/ros/humble 和本机 Unitree 消息工作空间后执行。
# tools/ros2_export.py 直接从源码运行，ROS Python 无需安装 h1-2-viewer。
python3 tools/ros2_export.py --config models/custom/h1_2/viewer.yaml \
  | conda run --no-capture-output -n h1_2_viewer h1-2-viewer \
      --source jsonl --config models/custom/h1_2/viewer.yaml
```

ROS 端只需要 rclpy、对应消息包、NumPy 和 PyYAML；不导入 Rerun。
本次验证了编解码及隔离路径，未在另一台系统 ROS 3.10 主机实际连接机器人。
也可把采集端 stdout 存为 `.jsonl`，随后 `h1-2-viewer --source jsonl --input recording.jsonl ...`。
JSONL 用于轻量联调；高频大点云长期录制仍应使用 rosbag/MCAP，而不是文本传输。

## 当前显示边界

局部模型视图跟随骨盆原点；地图中的机器人位姿必须来自显式定位/TF 接口，没有数据时不从躯干 IMU 推断。
雷达与模型在统一三维场景显示，尚不融合建图。没有反馈显示 WAITING，过期显示 STALE，并保留最后姿态。
手部只显示右 6、左 6 原始 q；尚未标定六通道到全部手指 URDF 关节角，未映射手指保持参考姿态。
URDF 本身配置的 mimic 关系可随已映射的驱动关节更新。

实时 ROS 时间轴是本机处理时刻，JSONL 使用采集端 elapsed；两者都不声称硬件级时间同步。
`--hz` 控制显示采样率，不是机器人控制频率，查看器不发送停止或运动命令。

## 控制面板、导航扩展

见 `docs/TELEMETRY.md` 和 [空间接口说明](docs/SPATIAL.md)。统一边界已支持机器人位姿、TF、物体位姿、抓取姿态和命名关节轨迹。
地图和路径要求显式坐标系；控制状态仅展示文本，不支持 execute/start/stop 等执行指令。
后续控制面板可把状态输入此边界，动作执行仍由独立执行层处理。

## 打包带走（包括你修改过的模型）

```bash
python tools_export_bundle.py --output /tmp/h1_2_viewer_bundle.tar.gz
```

按需输出迁移包，包含本目录代码、配置、环境配方、文档以及 `models/custom` 下的自定义模型。默认输出到系统临时目录，使用后清理，不长期保留测试打包产物。
不包含 Git 历史、官方缓存、上层 bridge、Conda 环境和 build/install/log。
**模型缓存与自定义副本默认 Git 忽略**，不能只 `git clone` 就期待取回本机模型修改；要用迁移包或显式版本管理它们。
当前更改尚未提交 Git，也未推送到远端。

## 验证

```bash
python -m unittest discover -s tests -v
h1-2-viewer --config models/custom/h1_2/viewer.yaml --check
h1-2-viewer --config models/custom/h1_2/viewer.yaml --demo --save /tmp/h1_2_demo.rrd
rerun rrd verify /tmp/h1_2_demo.rrd
```

修改查看器代码或 YAML 无需重新编译 bridge；`pip install -e .` 使用源码。
编辑模型后重启查看器。不要覆盖正在查看器中打开的同名录制文件。

设计参考及原先路径迁移说明见 `docs/ARCHITECTURE.md`。

空间数据的 Python、JSONL 和可选 ROS 字段详见 [docs/SPATIAL.md](docs/SPATIAL.md)。ROS 新订阅默认关闭，示例话题不是已存在的真机发布者。

## 布局参考

采用大场景配相机侧栏、调试信息独立页面的布局，参考 [Rerun 示例](https://rerun.io/examples) 和 [Foxglove 布局说明](https://docs.foxglove.dev/docs/visualization/layouts)。使用原生 Blueprint，不额外引入网页前端框架。
