"""Scripted navigation/grasp illustration. Synthetic data only; no ROS or physics."""
import numpy as np
import rerun as rr
from .task import STAGES, TaskState
from .spatial_data import Pose, ArmTrajectory

DURATION = 21.0


def phase(fraction):
    fraction = float(np.clip(fraction, 0, 1))
    index = min(int(fraction * len(STAGES)), len(STAGES) - 1)
    local = min(fraction * len(STAGES) - index, 1.0)
    smooth = local * local * (3 - 2 * local)
    distance = 2.4 * (1 - smooth) if index == 0 else 0.0
    return index, smooth, TaskState(STAGES[index][0], fraction, distance,
                                   target_visible=index >= 2,
                                   grasp_confirmed=index == 6 and local >= 0.75,
                                   simulated=True)


def body_pose(names, index, progress, t):
    q = np.zeros(len(names))
    targets = {'left_knee_joint': .3, 'right_knee_joint': .3,
               'left_hip_pitch_joint': -.15, 'right_hip_pitch_joint': -.15,
               'left_ankle_pitch_joint': -.15, 'right_ankle_pitch_joint': -.15,
               'left_shoulder_pitch_joint': .28, 'right_shoulder_pitch_joint': .28,
               'left_elbow_joint': .52, 'right_elbow_joint': .52}
    if index == 0:
        amplitude = .06 * np.sin(np.pi * progress)
        targets['left_hip_pitch_joint'] += amplitude * np.sin(t * 5)
        targets['right_hip_pitch_joint'] -= amplitude * np.sin(t * 5)
    reach = progress if index == 4 else (1.0 if index >= 5 else 0.0)
    lift = progress if index == 5 else (1.0 if index >= 6 else 0.0)
    targets['right_shoulder_pitch_joint'] += -.6 * reach - .7 * lift
    targets['right_elbow_joint'] += .5 * reach + .3 * lift
    for name, value in targets.items():
        if name in names:
            q[names[name]] = value
    return q


def wrist_tip(model, names, q):
    """Use official URDF joint transforms to align illustrative targets with the hand."""
    if 'right_wrist_yaw_link' not in model.links:
        return np.array([.45, -.25, .1])
    matrix = model.kinematics.tip_matrix(dict(zip(names, q)), 'right_wrist_yaw_link')
    return (matrix @ [.1, 0, 0, 1])[:3]


def camera_images(side, index, progress):
    """Stylized RGB and metric depth, not rendered/calibrated camera measurements."""
    h, w = 160, 240
    rgb = np.zeros((h, w, 3), dtype=np.uint8)
    rgb[:] = [32, 43, 58]
    rgb[75:] = [95, 83, 68]
    depth = np.full((h, w), 2.0, dtype=np.float32)
    depth[75:] = np.linspace(1.5, .45, h - 75)[:, None]
    size = 15 + (int(12 * progress) if index == 4 else (12 if index >= 5 else 0))
    center = 110 + {'left': 18, 'right': -10, 'head': 0}[side]
    top = 62 - (int(18 * progress) if index == 5 else (18 if index >= 6 else 0))
    if index >= 2:
        rgb[top:top+size*2, center-size:center+size] = [238, 160, 55]
        depth[top:top+size*2, center-size:center+size] = .65 if index < 4 else .35
        # Synthetic target outline; never presented as a detector output.
        rgb[top:top+2, center-size:center+size] = [65, 235, 160]
        rgb[top+size*2-2:top+size*2, center-size:center+size] = [65, 235, 160]
    # Blue strip marks every frame as synthetic even without a detection.
    rgb[:5] = [50, 130, 240]
    return rgb, depth


def initialize_scene(dashboard, target):
    goal = np.array([2.4, 0, 0])
    rr.log('navigation', rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    for entity in ('goal', 'table', 'travelled'):
        rr.log('navigation/' + entity, rr.CoordinateFrame(frame='map'), static=True)
    dashboard.tf('odom', Pose('map', [.2, -.1, 0], [0, 0, np.sin(.15), np.cos(.15)]), static=True, elapsed=0)
    x, y = np.meshgrid(np.linspace(-.5, 3.5, 35), np.linspace(-1.5, 1.5, 24))
    floor = np.column_stack([x.ravel(), y.ravel(), np.zeros(x.size)])
    dashboard.extensions.log_map(floor, frame_id='map')
    dashboard.extensions.log_path([[0, 0, .03], [1.2, 0, .03], [2.4, 0, .03]], frame_id='map')
    rr.log('navigation/goal', rr.Points3D([goal + [0, 0, .05]], radii=.09,
           colors=[80, 230, 130], labels=['Stop pose'], show_labels=True), static=True)
    center = target + [0, 0, -.095]
    rr.log('navigation/table', rr.Boxes3D(centers=[center + goal + [0, 0, 1]],
           sizes=[[.55, .65, .08]], colors=[110, 95, 75]), static=True)
    dashboard.tf('demo/head_optical', Pose('camera_link', [0, 0, 0], [-.5, .5, -.5, .5]), static=True, elapsed=0)
    rr.log('panels/perception', rr.TextDocument(
        '# Synthetic camera calibration\n\nRGB, metric depth, camera mounts, intrinsics and target outlines are illustrative.\n\nNo real camera, calibration, detector, locomotion, grasp policy or physics simulation is connected.'), static=True)
    rr.log('panels/navigation', rr.TextDocument(
        '# Synthetic environment\n\nFloor points, route, stop pose, robot marker and target are scripted in a shared demo frame.\n\nThis is not SLAM, navigation or sensor fusion. Lidar and head depth share the URDF scene.'), static=True)
    return goal


def run(dashboard, frames):
    names = {joint.name: i for i, joint in enumerate(dashboard.profile.joints)}
    target_q = body_pose(names, 4, 1, 0)
    target = wrist_tip(dashboard.model, names, target_q)
    dashboard.timestamp(0)
    goal = initialize_scene(dashboard, target)
    channels = dashboard.profile.settings['source']['hand_channels']
    previous, trace, plan_logged = None, [], False
    dt = DURATION / max(frames - 1, 1)
    for frame in range(frames):
        fraction = frame / max(frames - 1, 1)
        t = fraction * DURATION
        index, progress, task = phase(fraction)
        q = body_pose(names, index, progress, t)
        dq = np.zeros(len(names)) if previous is None else (q - previous) / dt
        previous = q.copy()
        with dashboard.spatial.batch():
            dashboard.body(q, dq, .4 * np.sin(q), elapsed=t)
            if frame % 4 == 0:
                dashboard.body_target(q + .015 * np.sin(t), elapsed=t)
            travel = 1 - task.distance / 2.4
            base = goal * travel
            trace.append(base + [0, 0, .04])
            map_from_base = np.array([base[0], base[1], 1.0, 1.0])
            odom_position = dashboard.spatial.graph.lookup('odom', 'map') @ map_from_base
            dashboard.robot_pose(Pose('odom', odom_position[:3], [0, 0, -np.sin(.15), np.cos(.15)]), elapsed=t)
            rr.log('navigation/travelled', rr.LineStrips3D([trace], colors=[100, 230, 170]))
            tip = wrist_tip(dashboard.model, names, q)
            attached = index >= 5
            object_position = (base if attached else goal) + (tip if attached else target) + [0, 0, 1]
            dashboard.object_pose('demo_object', Pose('map', object_position), size=[.07, .07, .11], elapsed=t)
            if index >= 3 and not plan_logged:
                tip_link = 'right_wrist_yaw_link' if 'right_wrist_yaw_link' in dashboard.model.links else dashboard.model.tree.root_link().name
                samples = [body_pose(names, 4, u, 0) for u in np.linspace(0, 1, 14)]
                trajectory = ArmTrajectory('right_approach', tuple(names), samples, np.linspace(0, 3, 14),
                                           dashboard.spatial.robot_frame, tip_link)
                dashboard.arm_trajectory(trajectory, elapsed=t)
                grasp = dashboard.spatial.kinematics.tip_matrix(dict(zip(names, target_q)), tip_link)
                dashboard.grasp_pose('right', Pose(dashboard.spatial.robot_frame, grasp[:3, 3]), elapsed=t)
                plan_logged = True
            hands = np.ones(channels)
            hands[:channels // 2] = 1 - (.8 * progress if index == 4 else (.8 if index >= 5 else 0))
            dashboard.hands(hands, elapsed=t)
            if frame % 4 == 0:
                dashboard.hand_target(hands, elapsed=t)
            rgb, depth = camera_images('head', index, progress)
            dashboard.head_depth(depth, [[180, 0, 120], [0, 180, 80], [0, 0, 1]], 'demo/head_optical', rgb=rgb, elapsed=t)
            for side in ('left', 'right'):
                dashboard.camera(side, *camera_images(side, index, progress), elapsed=t)
            theta = np.linspace(0, 2*np.pi, 360, endpoint=False)
            cloud = np.column_stack([2*np.cos(theta), 2*np.sin(theta), .2*np.sin(theta*3+t)])
            dashboard.cloud(cloud, 'lidar_link', elapsed=t)
        if frame % 10 == 0 or frame == frames - 1:
            dashboard.status(elapsed=t)
