"""Versioned, input-only JSON Lines transport for separate ROS/viewer environments."""
import json
from queue import Queue, Empty
import sys
import time
from threading import Thread, Event


def dispatch(dashboard, event):
    if not isinstance(event, dict):
        raise ValueError('Telemetry event must be a JSON object')
    if event.get('schema_version') != 1:
        raise ValueError('Unsupported telemetry schema_version')
    kind = event.get('type')
    elapsed = event.get('elapsed')
    if elapsed is not None:
        import math
        if not isinstance(elapsed, (int, float)) or not math.isfinite(elapsed) or elapsed < 0:
            raise ValueError('elapsed must be a finite nonnegative duration')
    if kind == 'body':
        dashboard.body(event['q'], event['dq'], event['tau'], elapsed=elapsed)
    elif kind == 'attitude':
        dashboard.attitude(event['quaternion_xyzw'], event.get('rpy'), elapsed=elapsed)
    elif kind == 'head_rgb':
        import numpy as np
        rgb = np.asarray(event['rgb'])
        if rgb.ndim != 3 or rgb.shape[2] != 3 or not np.all(np.isfinite(rgb)) or np.any(rgb < 0) or np.any(rgb > 255) or not np.all(rgb == np.floor(rgb)):
            raise ValueError('Head RGB must contain uint8-compatible HxWx3 values')
        dashboard.head_rgb(rgb.astype(np.uint8), elapsed=elapsed)
    elif kind == 'lidar':
        dashboard.cloud(event['points'], event['frame_id'], elapsed=elapsed)
    elif kind == 'body_target':
        dashboard.body_target(event['q'], elapsed=elapsed)
    elif kind == 'hand_target':
        dashboard.hand_target(event['q'], elapsed=elapsed)
    elif kind == 'hands':
        dashboard.hands(event['q'], elapsed=elapsed)
    elif kind in ('map', 'path'):
        dashboard.timestamp(elapsed)
        function = dashboard.extensions.log_map if kind == 'map' else dashboard.extensions.log_path
        function(event['points'], frame_id=event['frame_id'])
    elif kind in ('robot_pose', 'tf', 'object_pose', 'grasp_pose'):
        from .spatial_data import Pose
        pose = Pose(**event['pose'])
        if kind == 'robot_pose':
            dashboard.robot_pose(pose, child_frame=event.get('child_frame'), elapsed=elapsed)
        elif kind == 'tf':
            static = event.get('static', False)
            if not isinstance(static, bool):
                raise ValueError('TF static must be boolean')
            dashboard.tf(event['child_frame'], pose, static=static, elapsed=elapsed)
        elif kind == 'object_pose':
            dashboard.object_pose(event['object_id'], pose, size=event.get('size'), elapsed=elapsed)
        else:
            dashboard.grasp_pose(event['grasp_id'], pose, elapsed=elapsed)
    elif kind == 'arm_trajectory':
        from .spatial_data import ArmTrajectory
        dashboard.arm_trajectory(ArmTrajectory(**event['trajectory']), elapsed=elapsed)
    elif kind == 'task':
        from .task import TaskState
        dashboard.task(TaskState(**event['state']), elapsed=elapsed)
    elif kind == 'head_depth':
        import numpy as np
        rgb = event.get('rgb')
        if rgb is not None:
            raw = np.asarray(rgb)
            if not np.all(np.isfinite(raw)) or np.any(raw < 0) or np.any(raw > 255) or not np.all(raw == np.floor(raw)):
                raise ValueError('Invalid RGB pixels')
            rgb = raw.astype(np.uint8)
        dashboard.head_depth(event['depth'], event['intrinsics'], event['frame_id'], rgb=rgb,
                             stride=event.get('stride', 4), max_depth=event.get('max_depth', 8.0), elapsed=elapsed)
    elif kind == 'camera':
        import numpy as np
        raw = np.asarray(event['rgb'])
        if raw.ndim != 3 or not np.all(np.isfinite(raw)) or np.any(raw < 0) or np.any(raw > 255) or not np.all(raw == np.floor(raw)):
            raise ValueError('Camera RGB must contain uint8-compatible pixel values')
        dashboard.camera(event['side'], raw.astype(np.uint8), np.asarray(event['depth']), elapsed=elapsed)
    elif kind == 'control_status':
        dashboard.timestamp(elapsed)
        dashboard.extensions.log_control_status(event['text'])
    else:
        raise ValueError(f'Unsupported telemetry type: {kind}')


def run(dashboard, filename='-'):
    stream = sys.stdin if filename == '-' else open(filename, encoding='utf-8')
    queue, stop = Queue(maxsize=16), Event()

    def read():
        try:
            for line in stream:
                if stop.is_set():
                    return
                queue.put(line)
        finally:
            queue.put(None)
    Thread(target=read, daemon=True).start()
    last_elapsed, last_receive = None, time.monotonic()
    try:
        while True:
            try:
                line = queue.get(timeout=1.0)
            except Empty:
                elapsed = None if last_elapsed is None else last_elapsed + time.monotonic() - last_receive
                dashboard.status(elapsed=elapsed)
                continue
            if line is None:
                break
            try:
                event = json.loads(line)
                dispatch(dashboard, event)
                last_elapsed, last_receive = event.get('elapsed'), time.monotonic()
            except (ValueError, TypeError, KeyError) as exc:
                dashboard.errors['jsonl'] = str(exc)
                print(f'Rejected telemetry: {exc}', file=sys.stderr)
            dashboard.status(elapsed=last_elapsed)
    finally:
        stop.set()
        if stream is not sys.stdin:
            stream.close()
