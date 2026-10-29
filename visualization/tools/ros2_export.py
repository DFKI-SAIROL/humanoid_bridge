#!/usr/bin/env python3
"""ROS-side exporter (Python 3.10+). Does not import Rerun or robot controllers.

Run directly from the source/bundle; installation of h1-2-viewer is not required
in the ROS Python environment. Stdout contains only versioned JSON telemetry.
"""
import argparse
import json
from pathlib import Path
import sys
import time

# Keep this script executable from an unpacked bundle and from a different CWD.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from h1_2_viewer.configuration import load_profile


class JsonlSink:
    def __init__(self, stream=None):
        self.stream = stream or sys.stdout
        self.started = time.monotonic()
        self.errors = {}

    def _send(self, kind, **values):
        # Optional missing JointState velocity/effort travels as JSON null, not nonstandard NaN.
        def clean(value):
            import math
            if isinstance(value, dict):
                return {k: clean(v) for k, v in value.items()}
            if isinstance(value, (list, tuple)):
                return [clean(v) for v in value]
            if isinstance(value, float) and not math.isfinite(value):
                return None
            return value
        event = {'schema_version': 1, 'type': kind,
                 'elapsed': time.monotonic() - self.started, **values}
        self.stream.write(json.dumps(clean_event(event, clean), allow_nan=False) + '\n')
        self.stream.flush()

    def body(self, q, dq, tau):
        self._send('body', q=q.tolist(), dq=dq.tolist(), tau=tau.tolist())

    def attitude(self, quaternion_xyzw, rpy=None):
        self._send('attitude', quaternion_xyzw=quaternion_xyzw, rpy=rpy)

    def head_rgb(self, rgb):
        self._send('head_rgb', rgb=rgb.tolist())

    def body_target(self, q):
        self._send('body_target', q=q.tolist())

    def hand_target(self, q):
        self._send('hand_target', q=list(q))

    def cloud(self, xyz, frame_id):
        self._send('lidar', points=xyz.tolist(), frame_id=frame_id)

    def hands(self, positions):
        self._send('hands', q=list(positions))

    def robot_pose(self, pose, *, child_frame=None, elapsed=None):
        from dataclasses import asdict
        self._send('robot_pose', pose=asdict(pose), child_frame=child_frame)

    def tf(self, child_frame, pose, *, static=False, elapsed=None):
        from dataclasses import asdict
        self._send('tf', pose=asdict(pose), child_frame=child_frame, static=static)

    def object_pose(self, object_id, pose, *, size=None, elapsed=None):
        from dataclasses import asdict
        self._send('object_pose', object_id=object_id, pose=asdict(pose), size=size)

    def grasp_pose(self, grasp_id, pose, *, elapsed=None):
        from dataclasses import asdict
        self._send('grasp_pose', grasp_id=grasp_id, pose=asdict(pose))

    def arm_trajectory(self, trajectory, *, elapsed=None):
        from dataclasses import asdict
        self._send('arm_trajectory', trajectory=asdict(trajectory))

    def status(self):
        pass


def clean_event(event, clean):
    return {key: clean(value) for key, value in event.items()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--observe-client', action='store_true')
    args, ros_args = parser.parse_known_args()
    profile = load_profile(args.config)
    if args.observe_client:
        profile.settings['source']['body_target_topic'] = '/robot_cmd'
        profile.settings['source']['hand_target_topic'] = '/inspire/cmd'
    from h1_2_viewer.ros2_source import run
    try:
        run(JsonlSink(), profile, ros_args)
    except (KeyboardInterrupt, BrokenPipeError):
        pass


if __name__ == '__main__':
    main()
