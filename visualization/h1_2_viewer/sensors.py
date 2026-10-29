"""Calibrated sensor geometry, independent of ROS and transport."""
import numpy as np
import rerun as rr


def depth_points(depth, intrinsics, rgb=None, stride=4, max_depth=8.0):
    """Rectified depth in metres; optical axes X right, Y down, Z forward."""
    depth = np.asarray(depth, dtype=float)
    k = np.asarray(intrinsics, dtype=float)
    if depth.ndim != 2 or k.shape != (3, 3) or not np.all(np.isfinite(k)):
        raise ValueError('Depth must be HxW and intrinsics a finite 3x3 matrix')
    if k[0, 0] <= 0 or k[1, 1] <= 0 or not np.allclose(k[2], [0, 0, 1]) or k[0, 1] != 0 or k[1, 0] != 0:
        raise ValueError('Expected rectified pinhole intrinsics (no skew)')
    if isinstance(stride, bool) or not isinstance(stride, int) or stride < 1 or not np.isfinite(max_depth) or max_depth <= 0:
        raise ValueError('Invalid depth sampling limits')
    v, u = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    z = depth[::stride, ::stride]
    valid = np.isfinite(z) & (z > 0) & (z <= max_depth)
    points = np.stack(((u-k[0, 2])*z/k[0, 0], (v-k[1, 2])*z/k[1, 1], z), axis=-1)[valid]
    colors = [240, 175, 70]
    if rgb is not None:
        rgb = np.asarray(rgb)
        if rgb.shape != (*depth.shape, 3) or rgb.dtype != np.uint8:
            raise ValueError('RGB must be registered to depth, matching HxWx3 uint8')
        colors = rgb[::stride, ::stride][valid]
    return points, colors


class SensorScene:
    """Separate selectable layers, using the shared named-frame graph."""
    def __init__(self, spatial):
        self.spatial = spatial
        self.latest = {}
        self.dirty = set()
        self.last_status = None
        self.logged_frames = {}

    def cloud(self, layer, xyz, frame_id, colors, radius=.012):
        frame_id = self.spatial.resolve(frame_id)
        xyz = np.asarray(xyz, dtype=float)
        if xyz.ndim != 2 or xyz.shape[1] != 3 or not np.all(np.isfinite(xyz)):
            raise ValueError('Expected finite Nx3 sensor points')
        self.latest[layer] = (xyz, frame_id, colors, radius)
        self.dirty.add(layer)
        self.refresh()

    def refresh(self):
        pending = []
        for layer in list(self.dirty):
            xyz, source, colors, radius = self.latest[layer]
            path = 'robot/sensors/' + layer
            try:
                self.spatial.graph.lookup(self.spatial.robot_frame, source)
                if self.logged_frames.get(layer) != source:
                    rr.log(path, rr.CoordinateFrame(frame=source), static=True)
                    self.logged_frames[layer] = source
                rr.log(path, rr.Points3D(xyz, colors=colors, radii=radius))
                self.dirty.discard(layer)
            except ValueError as exc:
                rr.log(path, rr.Clear(recursive=True))
                pending.append(f'{layer}: {exc}')
        status = ('\n\n'.join(pending) if pending else
                  ('Sensor frames connected.' if self.latest else 'Waiting for sensor data.'))
        if status != self.last_status:
            rr.log('panels/sensors', rr.TextDocument('# Sensors\n\n' + status +
                   '\n\nDepth: rectified metres + intrinsics + optical TF.'))
            self.last_status = status
