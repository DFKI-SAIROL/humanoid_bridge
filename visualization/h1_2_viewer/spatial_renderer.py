"""Read-only spatial visualization. Pure contracts/TF math live in spatial_data.py."""
from contextlib import contextmanager
import numpy as np
import rerun as rr
from .spatial_data import Pose, ArmTrajectory, FrameGraph, frame, identifier, vector


class SpatialRenderer:
    def __init__(self, model, settings=None):
        self.kinematics = model.kinematics
        self.robot_frame = self.kinematics.root
        self.urdf_children = {'h1_2/' + j.find('child').get('link') for j in model.joints.values()}
        settings = settings or {}
        self.world_frame = frame(settings.get('world_frame', 'map'))
        self.aliases = {**self.kinematics.aliases, **settings.get('frame_aliases', {})}
        self.graph = FrameGraph(self.world_frame)
        self.graph.known.add(self.robot_frame)
        self.measured = {}
        self.entities, self.trajectories, self.pending = {}, {}, []
        self.robot_received = False
        self.logged_references = {}
        rr.log('navigation', rr.CoordinateFrame(frame=self.world_frame), static=True)
        rr.log('scene', rr.CoordinateFrame(frame=self.world_frame), static=True)
        rr.log('scene', rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
        self.update_body({}, refresh=False)
        for transform in settings.get('static_transforms', []):
            self.tf(transform['child'], Pose(transform['parent'], transform['position'],
                    transform['quaternion_xyzw']), static=True)
        self.last_panel = None
        self.refresh()

    def resolve(self, value):
        value, seen = frame(value), set()
        while value in self.aliases:
            if value in seen:
                raise ValueError('Frame alias cycle')
            seen.add(value)
            value = frame(self.aliases[value])
        return value

    def pose(self, value):
        if not isinstance(value, Pose):
            raise ValueError('Expected a Pose')
        return Pose(self.resolve(value.frame_id), value.position, value.quaternion_xyzw)

    def update_body(self, values, elapsed=None, refresh=True):
        expanded = self.kinematics.values(values)
        for name, xml in self.kinematics.model.joints.items():
            if xml.get('type') != 'fixed' and name not in expanded:
                continue  # Reference-pose geometry is not measured TF.
            child, pose = self.kinematics.local(name, expanded.get(name, 0))
            self.graph.set(child, pose, static=xml.get('type') == 'fixed', elapsed=elapsed, owner='urdf')
        self.measured = dict(values)
        if refresh:
            self.refresh()

    def tf(self, child_frame, pose, *, static=False, elapsed=None):
        child, normalized = self.resolve(child_frame), self.pose(pose)
        if child in self.urdf_children:
            raise ValueError('Internal URDF link TF is owned by joint feedback')
        was_imu = child in self.graph.edges and self.graph.edges[child][4] == 'imu'
        self.external_transform(child, normalized, static=static, elapsed=elapsed)
        if was_imu:
            rr.log('robot/localization', rr.Clear(recursive=False))
        rr.log('robot/external_tf/' + child, rr.Transform3D(translation=normalized.position,
               quaternion=normalized.quaternion_xyzw, parent_frame=normalized.frame_id, child_frame=child), static=static)
        self.refresh()

    def robot_pose(self, pose, *, child_frame=None, elapsed=None):
        child = self.resolve(child_frame) if child_frame else self.robot_frame
        if child in self.urdf_children:
            raise ValueError('Robot localization cannot override an internal URDF link')
        self.external_transform(child, self.pose(pose), elapsed=elapsed)
        self.robot_received = True
        # Explicit named-frame bridge also makes URDF available in world-origin Rerun views.
        normalized = self.pose(pose)
        rr.log('robot/localization', rr.Transform3D(translation=normalized.position,
               quaternion=normalized.quaternion_xyzw, parent_frame=normalized.frame_id, child_frame=child))
        self.refresh()

    def external_transform(self, child, pose, **options):
        # Localization/explicit TF wins over the display-only IMU fallback.
        old = self.graph.edges.get(child)
        fallback = old and old[4] == 'imu'
        if fallback:
            del self.graph.edges[child]
        try:
            self.graph.set(child, pose, **options)
        except ValueError:
            if fallback:
                self.graph.edges[child] = old
            raise

    def attitude(self, quaternion_xyzw, imu_frame, elapsed=None):
        from .attitude import root_orientation
        old = self.graph.edges.get(self.robot_frame)
        if old and old[4] != 'imu':
            return False
        # The IMU is on the torso, not the pelvis. Remove measured waist/mount rotation.
        root_from_imu = self.kinematics.tip_matrix(self.measured, self.resolve(imu_frame), require_complete=True)
        q = root_orientation(quaternion_xyzw, root_from_imu)
        pose = Pose(self.world_frame, [0, 0, 0], q)
        self.graph.set(self.robot_frame, pose, elapsed=elapsed, owner='imu')
        rr.log('robot/localization', rr.Transform3D(translation=pose.position,
               quaternion=q, parent_frame=self.world_frame, child_frame=self.robot_frame))
        self.refresh()
        return True

    def object_pose(self, object_id, pose, *, size=None):
        key = ('objects', identifier(object_id))
        pose = self.pose(pose)
        if size is not None:
            size = vector(size, 3)
            if min(size) <= 0:
                raise ValueError('Object size must be positive meters')
        self.entities[key] = (pose, size)
        self.refresh()

    def grasp_pose(self, grasp_id, pose):
        self.entities[('grasps', identifier(grasp_id))] = (self.pose(pose), None)
        self.refresh()

    def arm_trajectory(self, trajectory):
        if not isinstance(trajectory, ArmTrajectory):
            raise ValueError('Expected ArmTrajectory')
        if self.resolve(trajectory.frame_id) != self.robot_frame:
            raise ValueError('Joint trajectory frame_id must be the URDF root; its FK is root-relative')
        unknown = set(trajectory.joint_names) - self.kinematics.model.joints.keys()
        if unknown:
            raise ValueError(f'Unknown trajectory joints: {sorted(unknown)}')
        # Validate the whole plan before changing stored/rendered state.
        points = []
        for positions in trajectory.positions:
            state = {**self.measured, **dict(zip(trajectory.joint_names, positions))}
            matrix = self.kinematics.tip_matrix(state, trajectory.tip_link, require_complete=True)
            points.append(matrix[:3, 3])
        self.trajectories[trajectory.trajectory_id] = (trajectory, np.asarray(points))
        self.refresh()

    def log(self, path, reference, data):
        if self.logged_references.get(path) != reference:
            rr.log(path, rr.CoordinateFrame(frame=reference), static=True)
            self.logged_references[path] = reference
        rr.log(path, data)

    def axes(self, path, reference, matrix, label, size=None):
        colors = [[235, 85, 85], [80, 220, 130], [80, 155, 255]]
        p = matrix[:3, 3]
        self.log(path, reference, rr.Arrows3D(origins=[p] * 3,
                 vectors=(matrix[:3, :3] * .09).T, colors=colors))
        self.log(path + '/label', reference, rr.Points3D([p], radii=.012,
                 labels=[label], show_labels=True, colors=[255, 195, 75]))
        if size:
            corners = np.array([[x, y, z] for x in (-.5, .5) for y in (-.5, .5) for z in (-.5, .5)]) * size
            corners = corners @ matrix[:3, :3].T + p
            edges = [[corners[i], corners[j]] for i in range(8) for j in range(i + 1, 8)
                     if bin(i ^ j).count('1') == 1]
            self.log(path + '/bounds', reference, rr.LineStrips3D(edges, colors=[245, 165, 60]))

    @contextmanager
    def batch(self):
        """Refresh once per telemetry batch; always restore state on errors."""
        previous = getattr(self, 'defer_refresh', False)
        self.defer_refresh = True
        try:
            yield
        finally:
            self.defer_refresh = previous
            if not previous:
                self.refresh()

    def refresh(self):
        if getattr(self, "defer_refresh", False):
            return
        self.pending = []
        views = [('robot/spatial', self.robot_frame)]
        try:
            matrix = self.graph.lookup(self.world_frame, self.robot_frame)
            edge = self.graph.edges.get(self.robot_frame)
            if edge and edge[4] == 'imu':
                localization = 'IMU ORIENTATION ONLY · position unknown'
            else:
                self.axes('navigation/spatial/robot_pose', self.world_frame, matrix, 'Robot pose')
                localization = 'CONNECTED'
        except ValueError as exc:
            localization = 'WAITING FOR TF'
            self.pending.append(str(exc))
        for (kind, name), (pose, size) in self.entities.items():
            for root, reference in views:
                try:
                    matrix = self.graph.pose_matrix(pose, reference)
                    self.axes(f'{root}/{kind}/{name}', reference, matrix,
                              f'{kind[:-1]}: {name}', size)
                except ValueError as exc:
                    self.pending.append(f'{kind}/{name}: {exc}')
        for name, (trajectory, points) in self.trajectories.items():
            for root, reference in views:
                try:
                    matrix = self.graph.lookup(reference, self.robot_frame)
                    transformed = points @ matrix[:3, :3].T + matrix[:3, 3]
                    self.log(f'{root}/trajectories/{name}', reference,
                             rr.LineStrips3D([transformed], colors=[60, 210, 240]))
                    self.log(f'{root}/trajectories/{name}/end', reference,
                             rr.Points3D([transformed[-1]], radii=.015,
                             labels=[f'PLAN {name} | {trajectory.time_from_start[-1]:.2f}s | no execution'], show_labels=True))
                except ValueError as exc:
                    self.pending.append(f'trajectory/{name}: {exc}')
        lines = ['# Spatial telemetry', f'World: `{self.world_frame}` · Robot root: `{self.robot_frame}` · {localization}',
                 f'External TF edges: {sum(edge[4] != "urdf" for edge in self.graph.edges.values())}',
                 f'Objects/grasps: {len(self.entities)} · Plans: {len(self.trajectories)}',
                 'Axes: red X / green Y / blue Z. Joint trajectories are FK previews only.',
                 'Latest transforms only; no interpolation, sensor synchronization or control.']
        lines += ['\n| Parent | Child | Type |', '|---|---|---|']
        lines += [f'| {parent} | {child} | {"static" if static else "dynamic"} |'
                  for child, (parent, _, static, _, owner) in self.graph.edges.items() if owner != 'urdf']
        lines += ['\nWaiting for TF: ' + text for text in dict.fromkeys(self.pending)]
        text = '\n\n'.join(lines)
        if text != getattr(self, 'last_panel', None):
            rr.log('panels/spatial', rr.TextDocument(text, media_type='text/markdown'))
            self.last_panel = text
