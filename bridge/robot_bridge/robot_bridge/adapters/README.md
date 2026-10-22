# Adapters

An adapter makes an external controller publish `/robot_cmd` instead of
opening its own `rt/lowcmd`. The bridge then applies its own checks,
interpolation, 1 kHz publish and watchdog.

## The contract

| Rule | Why it bites |
|---|---|
| Match the controller's constructor and method names exactly | It is a drop-in; the caller is not modified |
| Never create an `rt/lowcmd` publisher, not even unused | `checkExternalPublisher_` counts publishers, not traffic |
| Reach the wire only through the injected client | Keeps the adapter testable and transport-agnostic |
| No controller import at module level | This repo does not depend on the controller |

The client is `robot_bridge.cmd_client.RobotCmdClient` or
`robot_bridge.robot_client.RobotClient`; both expose `send_cmd`.

## Requirements

```bash
source "$SIMPLE_VENV"/bin/activate       # the controller's venv
source /opt/ros/humble/setup.bash && source <workspace>/install/setup.bash
```

## GEAR WBC (`gear_wbc.py`)

Replaces `decoupled_wbc.control.envs.g1.utils.command_sender.BodyCommandSender`.

### 1. Launcher

```python
import rclpy
from robot_bridge.cmd_client import RobotCmdClient
from robot_bridge.adapters.gear_wbc import install

rclpy.init()
node = rclpy.create_node("gear_wbc_bridge")
client = RobotCmdClient(node, num_dof=29, control_frequency=50.0, interpolation_order=0.0)
install(client)  # before building the env
```

`install()` rebinds `g1_body.BodyCommandSender`, which is where `G1Body`
resolves it. Pass `module=` to target a different module.

### 2. Start control, then run

```python
client.start_control(default_position=[0.0] * 29)  # ramps over 2 s, blocks
```

The bridge drops every `/robot_cmd` until this returns.

### 3. Verify

```bash
tools/test.sh --unit
ros2 topic echo /robot_cmd          # 29 motors, duration 0.02
```

End to end against the real bridge, no hardware:
`bridge/robot_bridge/tests/integration/README.md`.

### Mapping

`send_command(cmd_q, cmd_dq, cmd_tau)` takes three `(29,)` joint-order arrays
and reproduces GEAR's loop exactly:

```python
for i in range(NUM_MOTORS):
    motor, joint = JOINT2MOTOR[i], MOTOR2JOINT[i]
    q[motor] = DEFAULT_MOTOR_ANGLES[motor] if joint == -1 else cmd_q[joint]
```

`kp`/`kd` are motor-indexed from `MOTOR_KP`/`MOTOR_KD`; a short list leaves the
tail at 0. `duration` is a constructor argument (default 0.02) because the
config dict carries no rate.

## GEAR hands (`gear_hands.py`)

Replaces `decoupled_wbc.control.envs.g1.utils.command_sender.HandCommandSender`,
which `g1_hand.py` resolves from its own namespace.

```python
from robot_bridge.adapters.gear_hands import install

install(client)                 # before building the env
client.start_hand_control()     # separate from start_control
```

Pass-through, no remap: `send_command` takes DDS order and `/hand_cmd/<side>`
is DDS order. It does carry GEAR's own gains (kp `[2, 1...]`, kd `[0.5, 0.2...]`),
because it is a drop-in and its caller passes none.

A goal-order pose is a different interface — remap it with
`g1_stack.gear.goal.hand_from_pose(pose, side)` first. `docs/hands_path.md` has
the two orderings side by side.

## What breaks it

- `Detected N publishers on /lowcmd` and the bridge will not start: something
  still holds that topic. Unitree's controller needs
  `MotionSwitcherClient.ReleaseMode()`; GEAR's `BodyStateProcessor` does this
  itself when `ENV_TYPE == real`.
- `/robot_cmd` echoes but the robot does not move: `start_control` was never
  called, or it returned failure.
- Robot lurches on start: `start_control` moved to `default_position` over 2 s
  from wherever it stood. An empty list arms to `ready_q_`; any other wrong
  length is refused, and nothing moves.
- `start_control` refused on the G1: the remote is off (`estop.require_remote`),
  or another program writes `/lowcmd` (release it: L2 + R2).
- Robot freezes holding the last pose at full gains: watchdog. Control ends
  `duration + 0.2 s` after the last command unless `hold_position`.
- `ModuleNotFoundError: bridge_interface`: workspace not sourced.
- `No module named pytest`: the controller's venv has none. Use `tools/test.sh`,
  or `--python <interpreter>` to pick one that does.
- `g1_29dof` has identity `JOINT2MOTOR`/`MOTOR2JOINT` and no `-1`, so a
  mapping bug will not show on that config. The unit tests use a non-identity
  fixture instead.
- Hands need `start_hand_control`, not `start_control`; the two are independent.
- `HandCommandSender` opens its own `rt/dex3/<side>/cmd` publisher. The adapter
  must not, and the bridge cannot see it if it does: `checkExternalPublisher_`
  queries the ROS graph, and `unitree_sdk2py` publishers are not in it.
