# 全身控制 Client 与可视化并行运行

控制与显示是两个独立进程。可视化不导入控制 Client，不发送命令或调用控制服务。

```text
用户策略／上层程序
        ↓
H1_2WholeBodyController
  ├─ body.send_cmd → /robot_cmd → robot_bridge → 机器人
  └─ send_hand_cmd → /inspire/cmd → 手部服务 → 双手
                         │
                只读订阅控制目标
                         ↓
                   可视化进程 → 浏览器
                         ↑
          /lowstate、/inspire/state、雷达、相机、TF
```

控制 Client 可以单独运行；可视化按自己的频率观察最新数据。关闭浏览器或停止可视化不调用 stop_control，也不退出控制 Client。
可视化不放进控制回调或控制循环，不启动控制 Client／bridge／驱动。
订阅使用 sensor-data QoS 和每流最新值缓存，不追赶堆积命令；两边仍共享机器的 CPU / GPU / 网络资源。

## 启动方法

终端 A：保持用户原来的全身控制程序及环境，由用户自行启动和管理。无需为了可视化改写 send_cmd 或 send_hand_cmd。

终端 B：进入可接收相同 ROS domain / DDS 数据的查看器环境，加载已编译消息包：

```bash
conda activate ros2_humble
cd /path/to/workspace
source install/local_setup.bash
cd src/humanoid_bridge/visualization
h1-2-viewer --observe-client --serve --language zh
```

使用者需要先按自己的实际网络配置 DDS；可沿用仓库 setup_unitree.sh。上述命令不启动机器人。
`--observe-client` 仅开启 `/robot_cmd` 和 `/inspire/cmd` 的只读订阅。
网页默认仍为上方机器人主场景，下方头部／左手／右手 RGB。
Debug → Joints → position 页：实际关节位置与橙色 `target:` 曲线同图；主场景始终由实际反馈更新。
双手原始反馈和归一化目标分别放在两个子页，避免混淆单位；手指模型暂不跟随原始手通道。
Debug → Connection 显示反馈和目标各自的数据新鲜度与接收计数。看到目标值不代表 bridge 已接受或机器人已执行。

自己的模型／传感器配置仍通过 `--config YOUR_CONFIG.yaml` 加载，并可同时使用 `--observe-client`。
也可在 YAML 配置目标话题（适用于重命名或自定义 namespace）：

```yaml
source:
  body_target_topic: /robot_cmd
  hand_target_topic: /inspire/cmd
```

可复用 `h1_2_viewer/config/client_observer.yaml`。默认两个字段为 null，普通查看器不会新增消息依赖；
`--observe-client` 会覆盖这两个字段为上述标准话题，改话题时只用 YAML、不加此选项。
身体目标消息为 bridge_interface/RobotCmd；手部目标为 unitree_go/MotorCmds。
RobotCmd 按 YAML motor_index 映射，再应用与反馈相同的 scale / offset；双手顺序为右六、左六。
网页移除了 --enable-control、控制 HTTP 服务及所有动作按钮。

## 模拟与跨环境

```bash
h1-2-viewer --demo --serve --language zh
```

演示会生成明确标记的模拟反馈和略有偏差的目标曲线，不导入 ROS 或控制器。
新增 JSONL 事件 body_target / hand_target 均使用 schema_version=1、q、可选 elapsed；
body_target 的 q 已是按配置关节顺序排列的模型坐标，hand_target 为右六左六的 [0,1] 值。
纯 ROS 导出端支持两个目标流，可用包含 source.*_target_topic 的配置，或 --observe-client 启用。
因此控制环境 Python 3.10 和查看器 Python ≥3.11 仍能分开运行；导出端作为独立观察进程，不插入控制循环。

该实现使用接收时刻的 elapsed，不进行硬件时钟同步或严格的逐帧命令／反馈配对。
策略状态和轨迹仍通过已有只读扩展接口输入；本次没有迁移、启动或替换用户的 locomotion / policy。
