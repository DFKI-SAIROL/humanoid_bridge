"""Opt-in rectified head depth subscription. No camera driver is started."""
import numpy as np


def decode_rgb(msg):
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}
    count = channels.get(msg.encoding)
    if count is None:
        raise ValueError('Unsupported head RGB encoding')
    if msg.step < msg.width * count or len(msg.data) < msg.height * msg.step:
        raise ValueError('Truncated RGB image')
    pixels = np.ndarray((msg.height, msg.width, count), dtype=np.uint8, buffer=bytes(msg.data),
                        strides=(msg.step, count, 1))
    if count == 1:
        return np.repeat(pixels, 3, axis=2)
    pixels = pixels[:, :, :3]
    return pixels[:, :, ::-1].copy() if msg.encoding.startswith('bgr') else pixels.copy()


def decode_depth(msg):
    types = {'16UC1': ('u2', .001), '32FC1': ('f4', 1.)}
    if msg.encoding not in types:
        raise ValueError('Head depth encoding must be 16UC1 (mm) or 32FC1 (m)')
    dtype, scale = types[msg.encoding]
    dtype = np.dtype(('>' if msg.is_bigendian else '<') + dtype)
    if msg.step < msg.width * dtype.itemsize or len(msg.data) < msg.height * msg.step:
        raise ValueError('Truncated depth image')
    return np.ndarray((msg.height, msg.width), dtype=dtype, buffer=bytes(msg.data),
                      strides=(msg.step, dtype.itemsize)).astype(float) * scale


def attach(node, dashboard, profile):
    settings = profile.settings['head_camera']
    if not settings['depth_topic'] and not settings['info_topic'] and not settings['rgb_topic']:
        return
    if bool(settings['depth_topic']) != bool(settings['info_topic']):
        raise ValueError('Set both head_camera.depth_topic and info_topic')
    from sensor_msgs.msg import Image, CameraInfo
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.head_info = None
    node.head_pending = None
    node.head_pair = None
    from .rgbd import RgbdPairer
    node.head_sync = RgbdPairer(settings['rgbd_tolerance_ms']) if settings['colorize_depth'] else None
    def pair(kind, msg):
        matched = node.head_sync.add(kind, msg)
        if matched is not None:
            node.head_pair = matched
    def info(msg):
        with node.data_lock:
            node.head_info = msg
    def depth(msg):
        with node.data_lock:
            if node.head_sync is not None:
                pair('depth', msg)
            else:
                node.head_pending = msg
    def rgb(msg):
        with node.data_lock:
            node.head_rgb_pending = msg
            if node.head_sync is not None:
                pair('rgb', msg)
    if settings['depth_topic']:
        node.subscriptions_.extend([
            node.create_subscription(CameraInfo, settings['info_topic'], info, qos),
            node.create_subscription(Image, settings['depth_topic'], depth, qos)])
    if settings['rgb_topic']:
        node.subscriptions_.append(node.create_subscription(Image, settings['rgb_topic'], rgb, qos))


def drain(node, dashboard, profile):
    from contextlib import nullcontext
    with getattr(node, 'data_lock', nullcontext()):
        rgb = getattr(node, 'head_rgb_pending', None)
        node.head_rgb_pending = None
        msg = getattr(node, 'head_pending', None)
        info = getattr(node, 'head_info', None)
        matched = getattr(node, 'head_pair', None)
        colored_rgb = None
        if matched is not None:
            msg, colored_rgb = matched
            if info is not None:
                node.head_pair = None
        if msg is not None and info is not None:
            node.head_pending = None
    if rgb is not None and (colored_rgb is None or info is None):
        try:
            dashboard.head_rgb(decode_rgb(rgb))
            dashboard.errors.pop('head_rgb', None)
        except (ValueError, TypeError) as exc:
            dashboard.errors['head_rgb'] = str(exc)
    if msg is None or info is None:
        return
    try:
        if msg.header.frame_id != info.header.frame_id or (msg.width, msg.height) != (info.width, info.height):
            raise ValueError('Depth/CameraInfo frame and resolution must match')
        if any(abs(x) > 1e-9 for x in info.d):
            raise ValueError('Use rectified depth and matching zero-distortion CameraInfo')
        cfg = profile.settings['head_camera']
        dashboard.head_depth(decode_depth(msg), np.asarray(info.k).reshape(3, 3), msg.header.frame_id,
                             rgb=decode_rgb(colored_rgb) if colored_rgb is not None else None,
                             stride=cfg['stride'], max_depth=cfg['max_depth'])
        dashboard.errors.pop('head_camera', None)
        if colored_rgb is not None:
            dashboard.errors.pop('head_rgb', None)
    except (ValueError, TypeError) as exc:
        dashboard.errors['head_camera'] = str(exc)
