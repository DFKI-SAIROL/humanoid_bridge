"""Optional ROS message adapters. No subscriptions are made unless configured.

Conversions use duck-typed ROS messages and pure spatial contracts; no Rerun.
"""
from .spatial_data import Pose, ArmTrajectory


def pose_message(message, frame_id):
    p, q = message.position, message.orientation
    return Pose(frame_id, [p.x, p.y, p.z], [q.x, q.y, q.z, q.w])


def transform_message(message):
    t, q = message.transform.translation, message.transform.rotation
    return message.child_frame_id, Pose(message.header.frame_id, [t.x, t.y, t.z], [q.x, q.y, q.z, q.w])


def trajectory_message(message, config, root_frame):
    return ArmTrajectory(config['trajectory_id'], tuple(message.joint_names),
                         tuple(tuple(p.positions) for p in message.points),
                         tuple(p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in message.points),
                         message.header.frame_id or root_frame, config['tip_link'])


def attach(node, sink, profile):
    config = profile.settings['spatial']['ros']
    from xml.etree import ElementTree as ET
    xml = ET.parse(profile.urdf).getroot()
    internal_links = {j.find('child').get('link') for j in xml.findall('joint')}
    pending = {}
    node.spatial_pending = pending
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy

    def add(topic, msg_type, callback, qos=qos_profile_sensor_data):
        if topic:
            node.subscriptions_.append(node.create_subscription(msg_type, topic, callback, qos))

    def receive(kind, msg):
        with node.data_lock:
            pending[kind] = (kind, msg, False)

    def receive_tf(message, static):
        with node.data_lock:
            for transform in message.transforms:
                child = transform.child_frame_id.strip('/').removeprefix('h1_2/')
                child = profile.settings['spatial']['frame_aliases'].get(child, child).removeprefix('h1_2/')
                if child in internal_links:
                    continue
                pending[('tf', transform.child_frame_id)] = ('tf', transform, static)

    if config['tf_topic'] or config['tf_static_topic']:
        from tf2_msgs.msg import TFMessage
        add(config['tf_topic'], TFMessage, lambda msg: receive_tf(msg, False))
        add(config['tf_static_topic'], TFMessage, lambda msg: receive_tf(msg, True),
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
    if config['robot_pose_topic']:
        from nav_msgs.msg import Odometry
        add(config['robot_pose_topic'], Odometry, lambda msg: receive('robot_pose', msg))
    if config['object_pose_topic'] or config['grasp_pose_topic']:
        from geometry_msgs.msg import PoseStamped
        for kind in ('object_pose', 'grasp_pose'):
            add(config[kind + '_topic'], PoseStamped, lambda msg, key=kind: receive(key, msg))
    if config['arm_trajectory_topic']:
        from trajectory_msgs.msg import JointTrajectory
        add(config['arm_trajectory_topic'], JointTrajectory, lambda msg: receive('arm_trajectory', msg))


def drain(node, sink, profile):
    config = profile.settings['spatial']['ros']
    pending = getattr(node, 'spatial_pending', {})
    from contextlib import nullcontext
    with getattr(node, 'data_lock', nullcontext()):
        items = sorted(list(pending.values()), key=lambda item: {'tf': 0, 'robot_pose': 1}.get(item[0], 2))
        pending.clear()
    for kind, msg, static in items:
        try:
            if kind == 'tf':
                child, pose = transform_message(msg)
                sink.tf(child, pose, static=static)
            elif kind == 'robot_pose':
                if not msg.child_frame_id:
                    raise ValueError('Odometry child_frame_id is required')
                sink.robot_pose(pose_message(msg.pose.pose, msg.header.frame_id), child_frame=msg.child_frame_id)
            elif kind == 'object_pose':
                sink.object_pose(config['object_id'], pose_message(msg.pose, msg.header.frame_id))
            elif kind == 'grasp_pose':
                sink.grasp_pose(config['grasp_id'], pose_message(msg.pose, msg.header.frame_id))
            else:
                # Default URDF root name is known from profile model/config, not an assumed base_link.
                from xml.etree import ElementTree as ET
                xml = ET.parse(profile.urdf).getroot()
                children = {j.find('child').get('link') for j in xml.findall('joint')}
                roots = {link.get('name') for link in xml.findall('link')} - children
                if len(roots) != 1:
                    raise ValueError('Cannot determine unique URDF trajectory root')
                sink.arm_trajectory(trajectory_message(msg, config, roots.pop()))
            sink.errors.pop(kind, None)
        except (ValueError, TypeError, KeyError) as exc:
            sink.errors[kind] = str(exc)
