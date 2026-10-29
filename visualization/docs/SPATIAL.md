# 空间数据接口

左下角默认是**任务概览**，不是控制器：展示当前阶段、整体进度、距停止位置的距离及目标/抓取确认状态。
`Debug | joints` 是关节调试；新增 `Spatial / TF` 标签显示坐标系连接、外部 TF 列表、空间数据数量和等待原因。

本模块全部为观察接口，不创建命令发布者、不执行轨迹、不修改实测关节状态。

## 统一约定

- 长度：米；关节角：弧度；轨迹相对时间：秒。
- 所有 Pose 都是 `frame_id` 坐标系中的位置和方向；四元数严格使用 **[x,y,z,w]**。
- 四元数必须接近单位长度（误差不超过 1e-3）；只对舍入误差归一化，拒绝零或明显错误的四元数。
- TF 表示 **parent_from_child**：child 原点及坐标轴在 parent 中的位置/方向。Pose.frame_id 就是 parent。
- 世界坐标系由 `spatial.world_frame` 配置，默认 `map`。
- 机器人根坐标系从实际 URDF 读取，H1-2 为 `h1_2/pelvis`。URDF 原始 link 名（如 `pelvis`）可以直接使用，会映射到带命名空间的帧。
- `base_link` 不自动等于 `pelvis`。只有确认两者同原点、同方向时，才配置显式别名；否则应提供真实固定变换。
- 一个 child 只允许一个 parent。拒绝环路、修改静态 TF、坐标系父节点/权威冲突，以及带 elapsed 的倒序 TF。
- TF 是最新值图，**没有历史插值、时刻匹配、自动超时失效或硬件同步**。外部系统负责时间同步和传感器融合，不能把它当作完整 tf2 Buffer。

## Python API

这些方法调用在已经运行的 `Dashboard` 实例上；模块不依赖控制器。

```python
from h1_2_viewer.spatial_data import Pose, ArmTrajectory

# map -> odom（固定变换示例）
dashboard.tf("odom", Pose("map", [0.2, 0.0, 0.0]), static=True, elapsed=0)

# odom -> URDF root：机器人根节点的位姿。
dashboard.robot_pose(Pose("odom", [1.0, 0.0, 1.0]), elapsed=0.1)

# 手眼外参：示例数字不能用作真机标定。
dashboard.tf("right_camera", Pose("right_wrist_yaw_link", [0.04, 0, 0.03]),
             static=True, elapsed=0.1)

# 相机坐标系中的物体位姿；size 可省略，省略时只画坐标轴和标签。
dashboard.object_pose("cup", Pose("right_camera", [0, 0, 0.5]),
                      size=[0.07, 0.07, 0.12], elapsed=0.2)

# 抓取目标是末端的目标位姿，不是运动指令。
dashboard.grasp_pose("right", Pose("pelvis", [0.4, -0.2, 0.1]), elapsed=0.2)
```

以上例子的位置、方向、时间和尺寸仅说明格式。需要先接入真实 body 反馈，才能沿活动关节链将相机观测变换到身体/地图。
**URDF 的零位参考图形不会冒充实测 TF**：缺少关节反馈或外参时，物体数据会保留并显示等待，补齐后自动重投影。
传入 `map` 的物体即使没有关节反馈，仍可在地图视图中显示；身体视图需要定位链连接。

轨迹输入为命名关节的计划位置序列，不按数组索引猜关节：

```python
# joint_names：此次计划涉及的 URDF 关节名，positions：N×M。
# time_from_start：N 个递增时间，至少两个采样点。
trajectory = ArmTrajectory(
    trajectory_id="right_approach",
    joint_names=joint_names,
    positions=planned_joint_positions,
    time_from_start=planned_times,
    frame_id="pelvis",
    tip_link="right_wrist_yaw_link",
)
dashboard.arm_trajectory(trajectory, elapsed=1.0)
```

FK 预览用计划关节值加接收时的实测其他关节值。末端链上未包含在计划中的活动关节必须已有实测值，否则拒绝预览。
frame_id 必须是 URDF 根节点（或其显式别名），因为 FK 输出相对该根节点；不能把它随意标成 map 或 camera。
预览端点是 `tip_link`，不是默认假设的夹持中心；需要 TCP 时请在 URDF 中加入相应固定工具 link。
显示轨迹与末端路径不会向实际关节状态回写。时间数组用于标注计划时长，**不会在未来时间点自动执行或播放电机目标**。
不做碰撞检测、IK、动力学可行性或关节限位安全认证。

## JSONL v1

与现有 body/camera/task 共用单向 JSONL 输入。每行一个对象；`elapsed` 是共同起点的非负秒数。

```json
{"schema_version":1,"type":"tf","elapsed":0,"child_frame":"odom","static":true,"pose":{"frame_id":"map","position":[0.2,0,0],"quaternion_xyzw":[0,0,0,1]}}
{"schema_version":1,"type":"robot_pose","elapsed":0.1,"pose":{"frame_id":"odom","position":[1,0,1],"quaternion_xyzw":[0,0,0,1]}}
{"schema_version":1,"type":"object_pose","elapsed":0.2,"object_id":"cup","size":[0.07,0.07,0.12],"pose":{"frame_id":"map","position":[1.4,-0.2,1.1],"quaternion_xyzw":[0,0,0,1]}}
{"schema_version":1,"type":"grasp_pose","elapsed":0.2,"grasp_id":"right","pose":{"frame_id":"pelvis","position":[0.4,-0.2,0.1],"quaternion_xyzw":[0,0,0,1]}}
```

`arm_trajectory` 事件包含一个 `trajectory` 对象，字段与上述 `ArmTrajectory` 同名：
`trajectory_id`、`joint_names`、`positions`、`time_from_start`、`frame_id`、`tip_link`。
JSONL 图片/大数组适合联调，不建议用它长期传输高频大数据。

## 可选 ROS 适配

默认全部关闭，避免假定机器人真实话题。确认话题后，在自己的 YAML 中开启需要的项：

```yaml
spatial:
  world_frame: map
  frame_aliases: {}  # 例如 base_link: pelvis，必须先确认原点/方向确实一致
  ros:
    tf_topic: /tf
    tf_static_topic: /tf_static
    robot_pose_topic: /odom
    object_pose_topic: /perception/target_pose
    grasp_pose_topic: /planning/grasp_pose
    arm_trajectory_topic: /planning/arm_trajectory
    object_id: target
    grasp_id: right
    trajectory_id: right_arm_plan
    tip_link: right_wrist_yaw_link
```

这里的话题名称只是**配置示例**，不是本仓库已经存在的发布者。

| 配置 | ROS 类型 | 含义 |
|---|---|---|
| tf_topic | tf2_msgs/msg/TFMessage | 动态坐标变换，按 child 缓存，不会仅保留整批 TF 的最后一条 |
| tf_static_topic | tf2_msgs/msg/TFMessage | 静态外参，使用可靠、transient-local QoS |
| robot_pose_topic | nav_msgs/msg/Odometry | header.frame_id 是 parent，child_frame_id 必填；只读取 pose，不读 twist |
| object_pose_topic | geometry_msgs/msg/PoseStamped | 单个配置 object_id 的位姿 |
| grasp_pose_topic | geometry_msgs/msg/PoseStamped | 单个配置 grasp_id 的目标位姿 |
| arm_trajectory_topic | trajectory_msgs/msg/JointTrajectory | 读取 joint_names、positions 和 time_from_start |

JointTrajectory.header.frame_id 为空时明确使用 URDF 根节点；非空时必须与根节点/别名一致。
普通 arm 轨迹没有腰部等状态时，需先有 body 反馈，才能正确计算 FK。
内部 URDF link 的 TF 由关节反馈拥有；ROS 适配器跳过这些内部 link 的重复 TF，接入定位和额外传感器变换。
显式 Python/JSONL 尝试覆盖内部 URDF TF 会报错，避免两套来源争夺身体姿态。

相机位姿不能同时作为“map 下的自由节点”又作为“手腕下固定节点”重复挂父节点。定位系统应输出合适的根节点位姿，或在外部先完成转换。
订阅本身不发布任何控制命令。所有新 ROS 配置也适用于 Python 3.10 采集端 `tools/ros2_export.py` → JSONL → Rerun 的隔离链路。
当前 ROS 输入使用接收/处理时刻；未把不同硬件时钟强行当成已同步时间戳。

## 验证与当前限制

模拟 demo 通过同一组接口发送：带旋转的 map→odom、移动的机器人根位姿、手腕→相机外参、物体位姿、抓取目标和命名关节轨迹。
单元测试覆盖旋转/平移合成、逆变换、缺失/冲突/环路 TF、静态变更、倒序更新、未观测活动关节、消息转换、轨迹校验及计划/实测隔离。
机器人在地图中用位姿坐标轴标记；左上保留局部机器人视图。原始雷达仍独立显示，没有宣称实现定位、点云融合或 SLAM。
ROS 适配器已实现并以模拟消息验证；没有启动 ROS 节点或连接真机验证。

真实接入时再提供：实际 topic/type、frame 树及根节点定义、手眼标定、末端/TCP 定义、关节命名，以及各数据源时间基准。当前搭建和模拟验证无需这些真机资料。
