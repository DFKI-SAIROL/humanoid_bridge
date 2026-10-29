"""Portable YAML profile and explicit joint/motor mapping; no ROS or renderer imports."""
from copy import deepcopy
from dataclasses import dataclass
from importlib.resources import files
import math
import os
from pathlib import Path

import yaml


@dataclass(frozen=True)
class Joint:
    name: str
    motor_index: int
    group: str
    scale: float = 1.0
    offset: float = 0.0


@dataclass
class Profile:
    settings: dict
    joints: tuple
    urdf: Path
    package_paths: dict

    @property
    def groups(self):
        return {group: [i for i, joint in enumerate(self.joints) if joint.group == group]
                for group in dict.fromkeys(joint.group for joint in self.joints)}


def default_settings():
    return yaml.safe_load(files('h1_2_viewer').joinpath('config/default.yaml').read_text())


def _merge(base, patch, path='config'):
    if not isinstance(patch, dict):
        raise ValueError(f'{path} must be a mapping')
    for key, value in patch.items():
        if key not in base:
            raise ValueError(f'Unknown setting: {path}.{key}')
        if isinstance(base[key], dict) and key not in ('package_paths', 'frame_aliases'):
            _merge(base[key], value, f'{path}.{key}')
        else:
            base[key] = value


def resolve_path(value, base):
    path = Path(os.path.expandvars(str(value))).expanduser()
    if '$' in str(path):
        raise ValueError(f'Unresolved environment variable in path: {value}')
    return (path if path.is_absolute() else base / path).resolve()


def load_profile(config=None, urdf=None):
    from .assets import default_cache
    settings = default_settings()
    base = Path.cwd()
    if config:
        config = Path(config).expanduser().resolve()
        base = config.parent
        _merge(settings, yaml.safe_load(config.read_text()) or {})
    if settings['schema_version'] != 1:
        raise ValueError('Unsupported profile schema_version')
    joints = tuple(Joint(**j) for j in settings['joints'])
    if not joints:
        raise ValueError('At least one joint mapping is required')
    for joint in joints:
        if not joint.name or '/' in joint.name or not joint.group or '/' in joint.group:
            raise ValueError('Joint names and groups must be nonempty single path segments')
        if isinstance(joint.motor_index, bool) or not isinstance(joint.motor_index, int) or joint.motor_index < 0:
            raise ValueError('motor_index must be a nonnegative integer')
        if not math.isfinite(joint.scale) or joint.scale == 0 or not math.isfinite(joint.offset):
            raise ValueError('Joint scale must be finite/nonzero and offset finite')
    if len({j.name for j in joints}) != len(joints) or len({j.motor_index for j in joints}) != len(joints):
        raise ValueError('Joint names and motor indices must be unique')
    runtime = settings['runtime']
    if not math.isfinite(runtime['hz']) or not 0 < runtime['hz'] <= 120:
        raise ValueError('runtime.hz must be in (0, 120]')
    if not math.isfinite(runtime['stale_after']) or runtime['stale_after'] <= 0:
        raise ValueError('runtime.stale_after must be positive')
    for key in ('debug_hz', 'lidar_hz'):
        if not math.isfinite(runtime[key]) or not 0 < runtime[key] <= 120:
            raise ValueError(f'runtime.{key} must be in (0,120]')
    if isinstance(runtime['lidar_max_points'], bool) or not isinstance(runtime['lidar_max_points'], int) or runtime['lidar_max_points'] < 1:
        raise ValueError('runtime.lidar_max_points must be positive')
    if not math.isfinite(runtime['lidar_voxel_size']) or runtime['lidar_voxel_size'] < 0:
        raise ValueError('runtime.lidar_voxel_size must be nonnegative')
    if not isinstance(runtime['log_raw_lidar'], bool):
        raise ValueError('runtime.log_raw_lidar must be boolean')
    if settings['source']['type'] not in ('unitree', 'joint_states'):
        raise ValueError('source.type must be unitree or joint_states')
    channels = settings['source']['hand_channels']
    if not isinstance(channels, int) or channels <= 0 or channels % 2:
        raise ValueError('hand_channels must be a positive even integer')
    layout = settings['layout']
    for key in ('model_eye', 'model_target', 'background'):
        if len(layout[key]) != 3 or not all(math.isfinite(v) for v in layout[key]):
            raise ValueError(f'layout.{key} must have three finite values')
    for key in ('row_shares', 'column_shares'):
        if len(layout[key]) != 2 or not all(math.isfinite(v) and v > 0 for v in layout[key]):
            raise ValueError(f'layout.{key} must have two positive finite values')
    if not math.isfinite(layout['lidar_point_radius']) or layout['lidar_point_radius'] <= 0:
        raise ValueError('layout.lidar_point_radius must be positive')
    head = settings['head_camera']
    if not isinstance(head['colorize_depth'], bool):
        raise ValueError('head_camera.colorize_depth must be boolean')
    if head['colorize_depth'] and not (head['rgb_topic'] and head['depth_topic']):
        raise ValueError('Colorized depth requires registered standard ROS RGB and depth topics')
    if not math.isfinite(head['rgbd_tolerance_ms']) or not 0 <= head['rgbd_tolerance_ms'] <= 100:
        raise ValueError('head_camera.rgbd_tolerance_ms must be in [0,100]')
    if head['video_topic'] and head['rgb_topic']:
        raise ValueError('Choose head_camera.video_topic or rgb_topic, not both')
    if head['video_height'] not in (180, 360, 720):
        raise ValueError('head_camera.video_height must be 180, 360 or 720')
    for key in ('rgb_hz', 'depth_hz'):
        if not math.isfinite(head[key]) or not 0 < head[key] <= 60:
            raise ValueError(f'head_camera.{key} must be in (0,60]')
    if not isinstance(head['max_rgb_width'], int) or head['max_rgb_width'] < 1:
        raise ValueError('head_camera.max_rgb_width must be positive')
    if isinstance(head['stride'], bool) or not isinstance(head['stride'], int) or head['stride'] < 1:
        raise ValueError('head_camera.stride must be a positive integer')
    if not math.isfinite(head['max_depth']) or head['max_depth'] <= 0:
        raise ValueError('head_camera.max_depth must be positive')
    if bool(head['depth_topic']) != bool(head['info_topic']):
        raise ValueError('Head camera requires both depth_topic and info_topic')
    if not isinstance(settings['model']['show_collisions'], bool):
        raise ValueError('model.show_collisions must be boolean')
    epsilon = settings['model']['joint_epsilon_rad']
    if not math.isfinite(epsilon) or epsilon < 0:
        raise ValueError('model.joint_epsilon_rad must be nonnegative')
    limit = settings['model']['mesh_face_limit']
    if isinstance(limit, bool) or not isinstance(limit, int) or limit < 0:
        raise ValueError('model.mesh_face_limit must be a nonnegative integer (0 = original)')
    if settings['layout']['language'] not in ('zh', 'en'):
        raise ValueError('layout.language must be zh or en')
    tint = settings['model']['display_tint']
    if tint is not None and (len(tint) != 4 or not all(math.isfinite(v) and 0 <= v <= 1 for v in tint)):
        raise ValueError('model.display_tint must be null or RGBA values in [0,1]')
    from .spatial_data import frame
    if not isinstance(settings['attitude']['enabled'], bool):
        raise ValueError('attitude.enabled must be boolean')
    frame(settings['attitude']['imu_frame'])
    frame(settings['spatial']['world_frame'])
    for alias, target in settings['spatial']['frame_aliases'].items():
        frame(alias)
        frame(target)
    aliases = settings['spatial']['frame_aliases']
    for alias in aliases:
        seen = set()
        node = alias
        while node in aliases:
            if node in seen:
                raise ValueError('Frame alias cycle')
            seen.add(node)
            node = aliases[node]
    for name, value in settings['spatial']['ros'].items():
        if name.endswith('_topic') and value is not None and (not isinstance(value, str) or not value):
            raise ValueError(f'spatial.ros.{name} must be null or a nonempty topic')
    from .spatial_data import Pose
    children = set()
    for transform in settings['spatial']['static_transforms']:
        if set(transform) != {'parent', 'child', 'position', 'quaternion_xyzw'}:
            raise ValueError('Static TF requires parent, child, position and quaternion_xyzw')
        child = frame(transform['child'])
        Pose(transform['parent'], transform['position'], transform['quaternion_xyzw'])
        if child in children:
            raise ValueError('Duplicate static TF child')
        children.add(child)
    model_path = settings['model']['urdf'] or os.environ.get('H1_2_URDF')
    model_path = resolve_path(model_path, base) if model_path else default_cache() / 'h1_2_description/h1_2.urdf'
    if urdf:
        model_path = resolve_path(urdf, Path.cwd())
    packages = {name: resolve_path(path, base) for name, path in settings['model']['package_paths'].items()}
    return Profile(deepcopy(settings), joints, model_path, packages)
