"""PointCloud2-shaped object decoder; ROS message imports are unnecessary offline."""
import numpy as np


def cloud_xyz(message):
    if not message.width or not message.height:
        return np.empty((0, 3), dtype=np.float32)
    fields = {field.name: field for field in message.fields}
    formats, offsets = [], []
    byte_order = '>' if message.is_bigendian else '<'
    for name in ('x', 'y', 'z'):
        field = fields.get(name)
        if field is None or field.count != 1:
            raise ValueError(f'Missing scalar {name} field')
        types = {7: 'f4', 8: 'f8'}  # ROS PointField FLOAT32 / FLOAT64 wire constants.
        if field.datatype not in types:
            raise ValueError(f'Unsupported {name} datatype: {field.datatype}')
        formats.append(byte_order + types[field.datatype])
        offsets.append(field.offset)
    dtype = np.dtype(dict(names=['x', 'y', 'z'], formats=formats,
                         offsets=offsets, itemsize=message.point_step))
    if message.row_step < message.width * message.point_step:
        raise ValueError('PointCloud2 row_step is too small')
    points = np.ndarray((message.height, message.width), dtype=dtype,
                        buffer=message.data, strides=(message.row_step, message.point_step))
    xyz = np.stack([points[name].ravel() for name in ('x', 'y', 'z')], axis=1)
    return np.asarray(xyz[np.isfinite(xyz).all(axis=1)], dtype=np.float32)


def display_cloud(xyz, max_points=4000, voxel_size=.04):
    """Display-only voxel sampling and hard point budget; no source messages change."""
    xyz = np.asarray(xyz, dtype=np.float32)
    if xyz.ndim != 2 or xyz.shape[1] != 3 or not np.all(np.isfinite(xyz)):
        raise ValueError('Expected finite Nx3 lidar points')
    if max_points < 1 or voxel_size < 0:
        raise ValueError('Invalid display cloud limits')
    if voxel_size > 0 and len(xyz):
        cells = np.floor(xyz.astype(np.float64) / voxel_size)
        _, indices = np.unique(cells, axis=0, return_index=True)
        xyz = xyz[np.sort(indices)]
    if len(xyz) > max_points:
        xyz = xyz[np.linspace(0, len(xyz) - 1, max_points, dtype=int)]
    return xyz
