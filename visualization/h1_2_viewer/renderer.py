"""Transport-independent telemetry rendering. Does not import ROS or controllers."""
import time
import numpy as np
import rerun as rr
from .extensions import DashboardExtensions


class Dashboard:
    """Rendering and validation shared by live subscriptions and offline tests."""
    def __init__(self, model, profile, demo=False):
        self.profile = profile
        self.zh = profile.settings['layout']['language'] == 'zh'
        self.names = tuple(j.name for j in profile.joints)
        self.groups = profile.groups
        stale_after = profile.settings['runtime']['stale_after']
        self.model = model
        self.stale_after = stale_after
        self.demo = demo
        self.started = time.monotonic()
        self._images = {}
        self._rates = {}
        self.lidar_display_points = 0
        self.last = {}
        self.counts = {'body': 0, 'lidar': 0, 'hands': 0, 'body_target': 0, 'hand_target': 0, 'imu': 0,
                       'head_rgb': 0, 'head_depth': 0}
        self.errors = {}
        self.lidar_frame = None
        self.lidar_points = 0
        self.extensions = DashboardExtensions()
        self.timestamp(0.0)
        model.log_geometry()
        from .spatial_renderer import SpatialRenderer
        self.spatial = SpatialRenderer(model, profile.settings.get('spatial')) if hasattr(model, 'tree') else None
        from .sensors import SensorScene
        self.sensors = SensorScene(self.spatial) if self.spatial else None
        rr.log('lidar_raw', rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
        rr.log('panels/control', rr.TextDocument(
            'Reserved: external control panel / policy status.\n'
            'This viewer has no robot command publishers or service clients.'))
        rr.log('panels/navigation', rr.TextDocument(
            'Reserved: mapping/localization/navigation.\n'
            'DashboardExtensions.log_map/log_path accept Nx3 coordinates in map frame.\n'
            'Caller sets the Rerun elapsed timeline before logging.\n'
            'Unified scene uses explicit URDF/TF sensor frames.'))
        rr.log('panels/joints', rr.TextDocument('Waiting for configured state topic; no measured joint values yet.'))
        rr.log('panels/attitude', rr.TextDocument(
            'Waiting for body IMU; no orientation/position measured yet.' if profile.settings['attitude']['enabled']
            else 'Body IMU display disabled in profile.'))
        from .i18n import translator
        tr = translator(profile.settings)
        for path, names in [('roll_pitch', [tr('Roll'), tr('Pitch')]), ('yaw', [tr('Yaw')]),
                            ('quaternion', ['x', 'y', 'z', 'w'])]:
            rr.log('imu/' + path, rr.SeriesLines(names=names), static=True)
        for metric in ('position', 'velocity', 'torque'):
            for group, indices in self.groups.items():
                rr.log(f'joints/{metric}/{group}', rr.SeriesLines(
                    names=[self.names[i].removesuffix('_joint') for i in indices]), static=True)
        rr.log('panels/task', rr.TextDocument(
            '# Task overview\n\nWaiting for task telemetry.\n\nNavigation, stabilization, perception, planning, grasp and confirmation are not connected.\n\nThis viewer does not execute actions.'))
        rr.log('panels/perception', rr.TextDocument(
            'Head depth uses calibrated intrinsics and optical TF. Demo intrinsics and optical mount are synthetic.'))
        for side in ('left', 'right'):
            for kind in ('rgb', 'depth'):
                rr.log(f'cameras/{side}/{kind}/waiting', rr.Points2D([[120, 80]],
                       labels=['WAITING FOR CAMERA DATA'], show_labels=True))
        for group, indices in self.groups.items():
            rr.log(f'commands/body/{group}', rr.SeriesLines(
                names=['target: ' + self.names[i] for i in indices], colors=[240, 175, 70]), static=True)
        self.status()

    def attitude(self, quaternion_xyzw, rpy=None, elapsed=None):
        """Orientation-only fallback; never estimates position or overrides localization."""
        if not self.profile.settings['attitude']['enabled'] or self.demo or self.spatial is None:
            return
        from .spatial_data import Pose, vector
        q = Pose('imu_world', [0, 0, 0], quaternion_xyzw).quaternion_xyzw
        angles = vector(rpy, 3) if rpy is not None else None
        self.timestamp(elapsed)
        applied = self.spatial.attitude(q, self.profile.settings['attitude']['imu_frame'], elapsed)
        self.accept('imu')
        if self.due('imu_debug', self.profile.settings['runtime']['debug_hz'], elapsed):
            # Euler values are displayed in degrees; quaternion components are unitless.
            if angles is None:
                rotation = Pose('imu_world', [0, 0, 0], q).matrix()
                angles = (np.arctan2(rotation[2, 1], rotation[2, 2]),
                          np.arcsin(np.clip(-rotation[2, 0], -1, 1)),
                          np.arctan2(rotation[1, 0], rotation[0, 0]))
            degrees = np.degrees(angles)
            rr.log('imu/roll_pitch', rr.Scalars(degrees[:2]))
            rr.log('imu/yaw', rr.Scalars(degrees[2:]))
            rr.log('imu/quaternion', rr.Scalars(q))
            title = '# 身体 IMU' if self.zh else '# Body IMU'
            note = ('仅姿态 · 位置／离地高度未知；航向不代表地图北向。' if self.zh else
                    'Orientation only; position/height unknown; yaw is not a map heading.')
            state = ('姿态已应用' if self.zh else 'Orientation applied') if applied else (
                    '使用外部定位／TF，IMU 不覆盖' if self.zh else 'External localization/TF owns root; IMU not applied')
            lines = [title, state, note, f'IMU frame: {self.profile.settings["attitude"]["imu_frame"]}',
                     'quaternion xyzw: ' + str([round(float(v), 5) for v in q])]
            if angles is not None:
                lines.append('RPY (deg): ' + str(np.round(np.degrees(angles), 2).tolist()))
            rr.log('panels/attitude', rr.TextDocument('\n\n'.join(lines), media_type='text/markdown'))

    def task(self, state, elapsed=None):
        from .task import TaskState
        if not isinstance(state, TaskState):
            raise ValueError('Expected TaskState telemetry')
        self.timestamp(elapsed)
        rr.log('panels/task', rr.TextDocument(state.markdown(), media_type='text/markdown'))

    def robot_pose(self, pose, *, child_frame=None, elapsed=None):
        self.timestamp(elapsed)
        self.spatial.robot_pose(pose, child_frame=child_frame, elapsed=elapsed)

    def tf(self, child_frame, pose, *, static=False, elapsed=None):
        self.timestamp(elapsed)
        self.spatial.tf(child_frame, pose, static=static, elapsed=elapsed)
        self.sensors.refresh()

    def object_pose(self, object_id, pose, *, size=None, elapsed=None):
        self.timestamp(elapsed)
        self.spatial.object_pose(object_id, pose, size=size)

    def grasp_pose(self, grasp_id, pose, *, elapsed=None):
        self.timestamp(elapsed)
        self.spatial.grasp_pose(grasp_id, pose)

    def arm_trajectory(self, trajectory, *, elapsed=None):
        self.timestamp(elapsed)
        self.spatial.arm_trajectory(trajectory)

    def camera(self, side, rgb, depth, elapsed=None):
        if side not in ('left', 'right'):
            raise ValueError('Camera side must be left or right')
        rgb, depth = np.asarray(rgb), np.asarray(depth)
        if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError('RGB must be an HxWx3 uint8 array')
        if depth.shape != rgb.shape[:2] or not np.all(np.isfinite(depth)) or np.any(depth <= 0):
            raise ValueError('Depth must match RGB dimensions and contain finite positive meters')
        self.timestamp(elapsed)
        for kind in ('rgb', 'depth'):
            rr.log(f'cameras/{side}/{kind}/waiting', rr.Clear(recursive=False))
        self.image(f'cameras/{side}/rgb/image', rgb)
        self.image(f'cameras/{side}/depth/image', depth, depth=True)

    def image(self, path, pixels, *, depth=False):
        pixels = np.asarray(pixels)
        previous = self._images.get(path)
        if previous is not None and previous.dtype == pixels.dtype and np.array_equal(previous, pixels, equal_nan=True):
            return
        self._images[path] = pixels.copy()
        rr.log(path, rr.DepthImage(pixels.astype(np.float32), meter=1) if depth else rr.Image(pixels))

    def head_depth(self, depth, intrinsics, frame_id, *, rgb=None, stride=4, max_depth=8.0, elapsed=None):
        self.accept('head_depth')
        if rgb is not None:
            self.head_rgb(rgb, elapsed=elapsed)
        if not self.due('head_depth', self.profile.settings['head_camera']['depth_hz'], elapsed):
            return
        from .sensors import depth_points
        points, colors = depth_points(depth, intrinsics, rgb, stride, max_depth)
        self.timestamp(elapsed)
        self.sensors.cloud('head_depth', points, frame_id, colors)
        self.image('cameras/head/depth', np.asarray(depth, dtype=np.float32), depth=True)

    def head_rgb(self, rgb, elapsed=None):
        rgb = np.asarray(rgb)
        if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError('Head RGB must be an HxWx3 uint8 array')
        self.accept('head_rgb')
        if not self.due('head_rgb', self.profile.settings['head_camera']['rgb_hz'], elapsed):
            return
        limit = self.profile.settings['head_camera']['max_rgb_width']
        if rgb.shape[1] > limit:
            # Only the 2D display image is resized; source calibration/depth are untouched.
            from PIL import Image
            rgb = np.asarray(Image.fromarray(rgb).resize((limit, max(1, round(rgb.shape[0]*limit/rgb.shape[1])))))
        self.timestamp(elapsed)
        rr.log('cameras/head/rgb', rr.Image(rgb).compress(jpeg_quality=80))

    def due(self, stream, hz, elapsed=None):
        now = time.monotonic() if elapsed is None else elapsed
        rates = getattr(self, '_rates', None)
        if rates is None:
            self._rates = rates = {}
        if now - rates.get(stream, -float('inf')) < 1 / hz - 1e-9:
            return False
        rates[stream] = now
        return True

    def timestamp(self, elapsed=None):
        rr.set_time('elapsed', duration=time.monotonic() - self.started if elapsed is None else elapsed)

    def accept(self, stream):
        self.last[stream] = time.monotonic()
        self.counts[stream] += 1
        self.errors.pop(stream, None)

    def body(self, positions, velocities, torques, elapsed=None):
        values = np.asarray([positions, velocities, torques], dtype=float)
        if values.shape != (3, len(self.names)) or not np.all(np.isfinite(values[0])) or np.any(np.isinf(values[1:])):
            raise ValueError('Body state must contain configured finite q/dq/tau values')
        self.timestamp(elapsed)
        if self.spatial is not None:
            self.spatial.update_body(dict(zip(self.names, values[0])), elapsed)
        self.model.log_body(values[0])
        if self.sensors:
            self.sensors.refresh()
        if self.due('debug', self.profile.settings['runtime']['debug_hz'], elapsed):
            for metric, row in zip(('position', 'velocity', 'torque'), values):
                for group, indices in self.groups.items():
                    rr.log(f'joints/{metric}/{group}', rr.Scalars(row[indices]))
            table = '| Joint | q (rad) | dq (rad/s) | torque (Nm) |\n|---|---:|---:|---:|\n'
            table += '\n'.join(f'| {name} | {q:.3f} | {dq:.3f} | {tau:.3f} |'
                               for name, q, dq, tau in zip(self.names, *values))
            rr.log('panels/joints', rr.TextDocument(table, media_type='text/markdown'))
        self.accept('body')

    def cloud(self, xyz, frame_id, elapsed=None):
        if not frame_id:
            raise ValueError('Point cloud frame_id is empty; refusing to mix unknown frames')
        xyz = np.asarray(xyz, dtype=float)
        if xyz.ndim != 2 or xyz.shape[1] != 3 or not np.all(np.isfinite(xyz)):
            raise ValueError('Expected finite Nx3 lidar points')
        self.lidar_frame = frame_id
        self.lidar_points = len(xyz)
        self.accept('lidar')
        runtime = self.profile.settings['runtime']
        if not self.due('lidar', runtime['lidar_hz'], elapsed):
            return
        from .pointcloud import display_cloud
        xyz = display_cloud(xyz, runtime['lidar_max_points'], runtime['lidar_voxel_size'])
        self.lidar_display_points = len(xyz)
        self.timestamp(elapsed)
        if runtime['log_raw_lidar']:
            rr.log('lidar_raw/points', rr.Points3D(xyz, radii=self.profile.settings['layout']['lidar_point_radius'], colors=[80, 200, 230]))
        if self.sensors:
            self.sensors.cloud('lidar', xyz, frame_id, [80, 200, 230])

    def body_target(self, positions, elapsed=None):
        q = np.asarray(positions, dtype=float)
        if q.shape != (len(self.names),) or not np.all(np.isfinite(q)):
            raise ValueError('Client targets must match mapped joints with finite positions')
        self.accept('body_target')
        if not self.due('body_target', self.profile.settings['runtime']['debug_hz'], elapsed):
            return
        self.timestamp(elapsed)
        for group, indices in self.groups.items():
            rr.log(f'commands/body/{group}', rr.Scalars(q[indices]))

    def hand_target(self, positions, elapsed=None):
        q = np.asarray(positions, dtype=float)
        if q.shape != (self.profile.settings['source']['hand_channels'],) or not np.all(np.isfinite(q)) or np.any((q < 0) | (q > 1)):
            raise ValueError('Expected right-then-left normalized hand targets in [0,1]')
        self.accept('hand_target')
        if not self.due('hand_target', self.profile.settings['runtime']['debug_hz'], elapsed):
            return
        self.timestamp(elapsed)
        half = len(q) // 2
        for i, value in enumerate(q):
            side = 'right' if i < half else 'left'
            rr.log(f'commands/hands/{side}/channel_{i % half}', rr.Scalars(float(value)))


    def hands(self, positions, elapsed=None):
        q = np.asarray(positions, dtype=float)
        if q.shape != (self.profile.settings['source']['hand_channels'],) or not np.all(np.isfinite(q)):
            raise ValueError('Hand feedback must match configured channel count (right then left)')
        self.accept('hands')
        if not self.due('hands', self.profile.settings['runtime']['debug_hz'], elapsed):
            return
        self.timestamp(elapsed)
        for index, value in enumerate(q):
            half = len(q) // 2
            side = 'right' if index < half else 'left'
            rr.log(f'hands/{side}/channel_{index % half}', rr.Scalars(float(value)))


    def status(self, elapsed=None):
        self.timestamp(elapsed)
        now = time.monotonic()
        lines = (['# H1-2 连接状态',
                  '**模拟演示 · 未连接机器人**' if self.demo else '**只读数据订阅**',
                  '模型随关节反馈更新，点云通过 URDF / TF 对齐。',
                  '双手曲线为原始通道反馈，手指模型保留参考姿态。',
                  f'雷达：接收 {self.lidar_points} 点 / 显示 {self.lidar_display_points} 点；坐标系：{self.lidar_frame or "等待数据"}']
                 if self.zh else ['# H1-2 connection',
                  '**Synthetic demo · no robot connection**' if self.demo else '**Read-only subscriptions**',
                  'Model follows joint feedback; sensor alignment uses URDF / TF.',
                  self.model.hand_reference_note,
                  f'Lidar: received {self.lidar_points} / displayed {self.lidar_display_points} points; frame={self.lidar_frame or "waiting"}'])
        source = self.profile.settings['source']
        enabled = {'body': bool(source['state_topic']), 'lidar': bool(source['lidar_topic']), 'hands': bool(source['hand_topic']), 'body_target': bool(source['body_target_topic']), 'hand_target': bool(source['hand_target_topic'])}
        enabled['imu'] = self.profile.settings['attitude']['enabled']
        head = self.profile.settings['head_camera']
        enabled['head_rgb'] = bool(head['video_topic'] or head['rgb_topic'])
        enabled['head_depth'] = bool(head['depth_topic'])
        for stream in self.counts:
            if not enabled[stream] and not self.demo:
                state = '已禁用' if self.zh else 'DISABLED in profile'
            elif stream not in self.last:
                state = '等待反馈' if self.zh else 'WAITING — no feedback'
            else:
                age = now - self.last[stream]
                state = (f'{"数据过期" if age > self.stale_after else "在线"} · {age:.1f} 秒前' if self.zh else f'{"STALE" if age > self.stale_after else "LIVE"} — age {age:.1f}s')
            label = {'body': '关节反馈', 'lidar': '雷达', 'hands': '双手反馈', 'body_target': '身体目标', 'hand_target': '双手目标', 'imu': '身体 IMU', 'head_rgb': '头部彩色图像', 'head_depth': '头部深度图'}[stream] if self.zh else stream
            lines.append(f'- {label}: {state}; {self.counts[stream]} samples')
        lines += [f'- Rejected {stream}: {error}' for stream, error in self.errors.items()]
        rr.log('panels/status', rr.TextDocument('\n\n'.join(lines), media_type='text/markdown'))

