"""Transport/render-independent spatial contracts and latest-value frame graph.

Distances: meters. Quaternions: xyzw, parent_from_child. No ROS or Rerun imports.
This graph does not interpolate timestamps or perform localization.
"""
from dataclasses import dataclass
import re
import numpy as np


def frame(value):
    if not isinstance(value, str) or not value.strip('/') or any(c.isspace() for c in value):
        raise ValueError('A nonempty, whitespace-free frame_id is required')
    return value.strip('/')


def identifier(value):
    if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z0-9_.-]+', value):
        raise ValueError('IDs must be nonempty letters/digits/underscore/dot/hyphen')
    return value


def vector(value, size):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f'Expected {size} finite values')
    return tuple(float(v) for v in result)


@dataclass(frozen=True)
class Pose:
    frame_id: str
    position: tuple
    quaternion_xyzw: tuple = (0., 0., 0., 1.)

    def __post_init__(self):
        object.__setattr__(self, 'frame_id', frame(self.frame_id))
        object.__setattr__(self, 'position', vector(self.position, 3))
        quaternion = np.asarray(vector(self.quaternion_xyzw, 4))
        norm = np.linalg.norm(quaternion)
        if abs(norm - 1) > 1e-3:
            raise ValueError('quaternion_xyzw must be unit length (x,y,z,w)')
        object.__setattr__(self, 'quaternion_xyzw', tuple(quaternion / norm))

    def matrix(self):
        x, y, z, w = self.quaternion_xyzw
        result = np.eye(4)
        result[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                          [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                          [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
        result[:3, 3] = self.position
        return result


@dataclass(frozen=True)
class ArmTrajectory:
    trajectory_id: str
    joint_names: tuple
    positions: tuple
    time_from_start: tuple
    frame_id: str
    tip_link: str

    def __post_init__(self):
        identifier(self.trajectory_id)
        frame(self.frame_id)
        frame(self.tip_link)
        names = tuple(self.joint_names)
        if not names or len(set(names)) != len(names) or any(not isinstance(n, str) or not n for n in names):
            raise ValueError('Trajectory joint_names must be nonempty and unique')
        q = np.asarray(self.positions, dtype=float)
        times = np.asarray(self.time_from_start, dtype=float)
        if q.ndim != 2 or q.shape[1] != len(names) or q.shape[0] < 2 or not np.all(np.isfinite(q)):
            raise ValueError('Trajectory needs at least two finite joint-position waypoints')
        if times.shape != (len(q),) or not np.all(np.isfinite(times)) or times[0] < 0 or np.any(np.diff(times) <= 0):
            raise ValueError('time_from_start must match waypoints and increase strictly from >=0 seconds')
        object.__setattr__(self, 'joint_names', names)
        object.__setattr__(self, 'positions', tuple(tuple(row) for row in q))
        object.__setattr__(self, 'time_from_start', tuple(times))
        object.__setattr__(self, 'frame_id', frame(self.frame_id))


class FrameGraph:
    def __init__(self, world_frame='map'):
        self.known = {frame(world_frame)}
        self.edges = {}  # child -> (parent, parent_from_child, static, timestamp, owner)

    def set(self, child, pose, *, static=False, elapsed=None, owner='external'):
        if not isinstance(static, bool):
            raise ValueError('TF static must be boolean')
        child, parent = frame(child), pose.frame_id
        if child == parent:
            raise ValueError('A TF cannot parent itself')
        if elapsed is not None and (not np.isfinite(elapsed) or elapsed < 0):
            raise ValueError('TF elapsed must be finite and nonnegative')
        matrix = pose.matrix()
        old = self.edges.get(child)
        if old:
            old_parent, old_matrix, old_static, old_time, old_owner = old
            if old_owner != owner or old_parent != parent:
                raise ValueError(f'Conflicting TF authority/parent for {child}')
            if old_static != static or (old_static and not np.allclose(matrix, old_matrix, atol=1e-7)):
                raise ValueError(f'Cannot change static TF {child}')
            if elapsed is not None and old_time is not None and elapsed < old_time:
                raise ValueError(f'Out-of-order TF for {child}')
        node = parent
        while node in self.edges:
            if node == child:
                raise ValueError('TF cycle detected')
            node = self.edges[node][0]
        if node == child:
            raise ValueError('TF cycle detected')
        self.edges[child] = (parent, matrix, static, elapsed, owner)
        self.known.update((parent, child))

    def _root(self, source):
        matrix = np.eye(4)
        while source in self.edges:
            parent, local, *_ = self.edges[source]
            matrix = local @ matrix
            source = parent
        return source, matrix

    def lookup(self, target, source):
        target, source = frame(target), frame(source)
        if target not in self.known or source not in self.known:
            raise ValueError(f'Missing TF frame: {source} -> {target}')
        root_a, a = self._root(target)
        root_b, b = self._root(source)
        if root_a != root_b:
            raise ValueError(f'Disconnected TF: {source} -> {target}')
        return np.linalg.inv(a) @ b

    def pose_matrix(self, pose, target):
        return self.lookup(target, pose.frame_id) @ pose.matrix()
