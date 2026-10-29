"""IMU conventions and rigid-body rotation math; no ROS or rendering imports."""
import numpy as np
from .spatial_data import Pose, vector


def unitree_attitude(message):
    """LowState IMU is wxyz; the viewer contract is world_from_imu, xyzw."""
    w, x, y, z = vector(message.imu_state.quaternion, 4)
    q = Pose('imu_world', [0, 0, 0], [x, y, z, w]).quaternion_xyzw
    return q, vector(message.imu_state.rpy, 3)


def matrix_quaternion(rotation):
    """Unit xyzw quaternion from an orthogonal rotation, stable near 180 degrees."""
    r = np.asarray(rotation)
    k = np.array([
        [r[0, 0]-r[1, 1]-r[2, 2], r[0, 1]+r[1, 0], r[0, 2]+r[2, 0], r[2, 1]-r[1, 2]],
        [r[0, 1]+r[1, 0], r[1, 1]-r[0, 0]-r[2, 2], r[1, 2]+r[2, 1], r[0, 2]-r[2, 0]],
        [r[0, 2]+r[2, 0], r[1, 2]+r[2, 1], r[2, 2]-r[0, 0]-r[1, 1], r[1, 0]-r[0, 1]],
        [r[2, 1]-r[1, 2], r[0, 2]-r[2, 0], r[1, 0]-r[0, 1], np.trace(r)]])
    _, eigenvectors = np.linalg.eigh(k)
    return tuple(eigenvectors[:, -1])


def root_orientation(quaternion_xyzw, root_from_imu):
    world_from_imu = Pose('imu_world', [0, 0, 0], quaternion_xyzw).matrix()[:3, :3]
    return matrix_quaternion(world_from_imu @ root_from_imu[:3, :3].T)
