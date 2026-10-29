# 解耦依据与迁移说明

## 查阅的案例

1. [Rerun 官方 Animated URDF](https://rerun.io/examples/robotics/animated_urdf)：静态模型只加载一次，关节状态通过动态变换驱动。本项目改为官方 `rerun.urdf.UrdfTree`，不再自维护 origin/axis/旋转合成和基础几何加载。
2. [Rerun 官方 ROS node 示例](https://github.com/rerun-io/rerun/blob/main/examples/python/ros_node/main.py)：ROS 消息在接入边界转换成可视化数据，而非让控制器承担显示职责。本项目进一步把 ROS 导入限制在可选实时适配器内。
3. [ROS robot_state_publisher](https://github.com/ros/robot_state_publisher/tree/humble)：URDF 描述和 JointState 状态分开。遵循同样的模型/状态分离，同时保留原始 Unitree 电机数组适配，标准 JointState 按名称匹配。
4. [Rerun Blueprint](https://rerun.io/docs/concepts/visualization/blueprints)：数据与界面布局分离。默认布局有自己的模块，视角与比例从 YAML 读取，用户保存的 .rbl 可以独立加载。

这些是职责划分的参考，而不是把演示项目直接当成真机控制栈。本次没有新增 SLAM、导航算法、policy 执行层或机器人控制。

## 原先的问题与现在的边界

| 原先 | 现在 |
|---|---|
| 查看器装在 whole_body_controller，依赖 robot_bridge CMake | 独立 pip 包，控制模块仅保留旧入口转发 |
| import whole_body_controller 会顺带导入控制客户端 | 控制类延迟导入，离线查看器完全不导入 ROS/控制器 |
| 向父目录搜索固定 assets 路径 | 默认有版本模型缓存，任意 YAML 支持相对模型路径 |
| 电机映射与曲线分组写在 Python 常量 | YAML joints 配置显式索引、名称、分组、方向、零位 |
| 自己解析 URDF 后手算变换，只支持 mesh | 使用官方 UrdfTree，支持官方 loader 的几何，mimic 更新明确处理 |
| 几何和材质混在模型文件改动中 | 可选显示 tint 只作用于临时 URDF，原文件不变 |
| ROS 消息与 Rerun 同进程、同 Python 版本 | 可选 ROS 适配器，或 Python 3.10 ROS → JSONL → Python ≥3.11 Rerun |
| 模型在仓库之外，复制源码会漏掉资源 | 固定模型清单/哈希、显式可编辑副本、包含模型的迁移包 |
| 修改 XML 与 URDF 的效果容易混淆 | 文档明确 MJCF 用于模拟器、URDF 用于当前查看器，二者不会自动双向同步 |

## 数据方向

```text
ROS / Unitree / JointState ──> 状态适配 ──┐
离线合成数据 ────────────────────────────┼─> 统一数据 ─> Rerun renderer + Blueprint
外部进程 JSONL ─────────────────────────┘                   ↑
                                                URDF + mesh + YAML
```

观察链路只接收数据，不向控制器回写。控制面板和导航均通过展示数据接入。
没有为了“以后可能需要”引入插件框架、微服务、数据库或远程控制协议。

## 当前本机路径

项目：`/home/kaixiang/sariol_ws/src/humanoid_bridge/visualization`。
可编辑模型：该目录 `models/custom/h1_2/h1_2_description`，配置 `models/custom/h1_2/viewer.yaml`。
最初用于下载的官方 Git checkout 已清理。可复现来源保留在 `h1_2_viewer/config/model.lock.json`，本地基线和可编辑模型分别位于 `models/upstream`、`models/custom`。
源码中的模型路径和配置没有写死 `/home/kaixiang` 或 `ros2_humble`。

本目录为现有 humanoid_bridge 仓库的一部分，可单独打包；尚未建立独立远程仓库或推送当前修改。

空间模块进一步分为 `spatial_data.py`（不依赖 ROS/Rerun 的契约与帧图）、`kinematics.py`（URDF FK 适配）、`spatial_renderer.py`（Rerun 叠加）与 `spatial_ros.py`（可选消息/订阅适配）。demo 和 JSONL 使用同样的公共入口，不再分别维护目标/轨迹的直接绘图实现。
