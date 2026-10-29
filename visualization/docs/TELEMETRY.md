# 单向观测接口 v1

这是展示数据协议，不接受运动命令。每行一个 JSON 对象，必需 `schema_version: 1` 和 `type`。
`elapsed` 为采集端单调时钟相对启动的秒数，允许省略；没有值时使用查看器本地时间。

```json
{"schema_version":1,"type":"body","elapsed":1.0,"q":[0.1],"dq":[0.0],"tau":[0.0]}
{"schema_version":1,"type":"lidar","elapsed":1.0,"points":[[1.0,2.0,0.3]],"frame_id":"livox_frame"}
{"schema_version":1,"type":"hands","elapsed":1.0,"q":[0,0,0,0,0,0,0,0,0,0,0,0]}
{"schema_version":1,"type":"map","elapsed":1.0,"points":[[1.0,2.0,0.0]],"frame_id":"map"}
{"schema_version":1,"type":"path","elapsed":1.0,"points":[[0,0,0],[1,0,0]],"frame_id":"map"}
{"schema_version":1,"type":"control_status","elapsed":1.0,"text":"External control panel not connected"}
```

body 示例仅演示字段；默认模型实际需要 27 个 q/dq/tau，顺序为 YAML `joints` 顺序。
q 是模型弧度，dq 是模型 rad/s；JSONL 上游应已完成电机索引和 scale/offset 转换，显示端不再转换。
tau 是原始估计力矩 Nm；缺失 dq/tau 用 null，不能用虚构零值代替。q 不允许缺失或非有限。
默认手部是右六、左六原始反馈，单位没有推断，不能直接当模型弧度。
map/path 是已转换至 `map` 的 Nx3 数据；原始雷达有独立传感器 frame_id，不自动与地图融合。
未知 schema/type、错误数组形状或坐标系会被拒绝并显示错误。

同进程可直接调用 `renderer.Dashboard.body/cloud/hands` 和 `extensions.DashboardExtensions`。
JSONL 是将相同数据边界跨进程传输的一种简单方式；没有网络监听、服务发现或反向控制通道。
大点云不建议长期使用 JSONL，应由后续二进制/MCAP 数据源实现相同渲染接口。

## 任务与双手相机输入

同进程使用 `Dashboard.task(TaskState(...), elapsed=...)` 和 `Dashboard.camera(side, rgb, depth, elapsed=...)`。
任务类型见 `task.py`，相机要求 RGB 为 H×W×3 uint8，depth 为同尺寸有限正值米制深度。
示例 JSONL 任务事件：

```json
{"schema_version":1,"type":"task","elapsed":9.0,"state":{"stage":"observe","progress":0.4,"distance":0.0,"target_visible":true,"grasp_confirmed":false,"simulated":false}}
```

相机事件字段为 `type: camera`、`side: left/right`、`rgb`（整数像素数组）和 `depth`（米制二维数组）。
阶段名称：`navigate`、`stabilize`、`observe`、`plan`、`grasp`、`lift`、`confirm`；未到确认阶段不能报告抓取确认。
这些事件只改变显示，不会触发运动、调用控制服务或加载 policy。真实 ROS 相机和任务发布者尚未接入。

## 空间数据

新增 `robot_pose`、`tf`、`object_pose`、`grasp_pose`、`arm_trajectory` 事件与同名 Python 方法；完整字段、坐标约定和 ROS 配置见 [SPATIAL.md](SPATIAL.md)。
这些方法只接收显示数据，不发送机器人命令。

## Client 目标旁路观察

新增 `body_target` / `hand_target` JSONL 事件，字段为 `schema_version: 1`、`type`、`q`、可选 `elapsed`。
身体 q 为配置关节顺序的模型坐标；手部 q 为右六左六 [0,1]。只更新目标曲线，不更新真实模型。
完整架构见 [Client 并行观察](CLIENT_OBSERVER.md)。
