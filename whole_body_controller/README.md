# H1-2 控制器与查看器入口

控制 API 保留在 `h1_2_controller.py`：`H1_2WholeBodyController` 组合身体控制客户端与 Inspire 手命令接口。
`hand_example.py` 是发送一次双手目标的控制示例，运行它会实际发送手部命令；`--help` 仅显示帮助。

Rerun 查看器现已解耦至仓库独立目录 [`../visualization`](../visualization/README.md)，通过 `pip install -e` 安装，不再依赖控制器的 CMake 安装。

```bash
# 激活可运行 Rerun 的 Python >=3.11 环境后
python -m pip install -e /path/to/humanoid_bridge/visualization
h1-2-models fetch
h1-2-viewer --demo
```

日常编辑模型使用 `visualization/models/custom/h1_2/viewer.yaml` 指定的 URDF/XML/meshes。
参见 [独立查看器的安装、迁移、模型修改说明](../visualization/README.md)。

旧 `python -m whole_body_controller.dashboard` 保留为兼容入口，但实现已转移到 `h1_2_viewer`。
`robot_model.py` 和 `dashboard_extensions.py` 也只保留导入转发，不再保存另一套实现。
旧 `lidar_viewer.py` 仍可只查看雷达，点云解析复用独立包的 `pointcloud.py`。

全身控制 Client 与查看器独立运行。Client 使用原有 body.send_cmd 和 send_hand_cmd，查看器通过 ROS 话题只读观察控制目标和真实反馈，不导入控制类。
使用 `h1-2-viewer --observe-client --serve --language zh` 同步查看；详见 [并行观察架构](../visualization/docs/CLIENT_OBSERVER.md)。
网页动作按钮和控制服务已移除。Unitree 的 enable_joystick 开关修复保留，默认关闭；原 H1/G1 遥控示例显式开启。
