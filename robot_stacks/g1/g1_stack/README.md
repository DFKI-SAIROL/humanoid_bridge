# g1_stack

The G1 deploy layer: joint state off DDS, assembled observations, and GEAR
whole-body-controller goals.

| Module | What it owns |
|---|---|
| `robot.py` | What the G1 is: joint counts, the 29-joint state layout |
| `joints/` | Reading arm and hand angles off Unitree DDS, and their fake |
| `observation/` | One validated frame of camera plus joints |
| `gear/` | Goal order, goal messages, the ZMQ publisher, its pacer, and the walk-policy status reader |
| `amo/` | AMO, psi0's lower-body policy (port of psi0 `real/teleop`), its gravity feedforward, its ZMQ goal |
| `nodes/` | `gear_controller`, `amo_controller`, `fixed_goals`, `joint_probe`, `dex3_probe` |

Depends on `robot_bridge` (the bridge's Python client) and `g1_camera`.
Nothing under `bridge/` depends on this — that rule is what keeps the bridge
layer separable.

## Direction of the two "gear" modules

They face opposite ways, so check which one you want:

- `g1_stack.gear` — *drives* GEAR: builds goals and publishes them over ZMQ.
- `robot_bridge.adapters.gear_wbc` — lets GEAR *send* joint commands into the
  bridge.

## The goal array

Three joint orderings disagree — the controller's, the robot's state order, and
any policy's action order. Build a pose by name, never by position:

```python
from g1_stack.gear import goal

pose = goal.pose_from_named({"left_elbow_joint": 0.5})  # (31,) float32
msg = goal.hold_goal(pose, target_time, base_height=0.75)
```

`goal.UPPER_BODY_JOINTS` is the 31 names in goal order. `goal.goal(pose, nav,
height, target_time)` is the general form.

## Verify

```bash
pytest robot_stacks/g1/g1_stack/tests/
~/github/psi0/.venv-psi/bin/python -m pytest robot_stacks/g1/g1_stack/tests/amo   # AMO against psi0's own code: torch, pinocchio
```

Everything, including the bridge against a fake robot: `tools/test.sh`.

## What breaks it

- **`target_time` is a deadline on `time.monotonic()`**, compared against the
  controller's *own* clock. Two machines do not share one, so the publisher and
  the control loop must be the same machine.
- **The controller wants 31 joints only with the waist enabled.** Without it it
  wants 28 and the waist belongs to the legs. Run the loop with `--enable-waist`.
- **Hand joint order is unconfirmed against hardware.** Move one left-hand
  finger and watch which half of the 14-vector changes.
- **`--interface sim` does not reach the bridge**: MuJoCo listens on `rt/lowcmd`.
- **`amo_controller` needs psi0's venv**: CUDA torch (the traced AMO allocates on `cuda:0`), `pin<4`, pyzmq.
  `pin` 4 pulls numpy 2, and numpy 2 breaks the ROS messages: install with `numpy==1.26.4` pinned.
