import collections
import numpy as np


def get_gravity_orientation(quaternion):
    w, x, y, z = np.asarray(quaternion, dtype=np.float64)
    return np.array([2 * (w*y - x*z), -2 * (w*x + y*z), 1 - 2 * (w*w + z*z)], dtype=np.float32)


def create_history(term_dims, history_length):
    return {name: collections.deque(
        [np.zeros(dim, dtype=np.float32) for _ in range(history_length)], maxlen=history_length)
        for name, dim in term_dims.items()}


def build_observation(terms, histories, term_dims, clip):
    for name, dim in term_dims.items():
        value = np.asarray(terms[name], dtype=np.float32)
        if value.shape != (dim,) or not np.all(np.isfinite(value)):
            raise ValueError(f"Invalid observation term: {name}")
        histories[name].append(value)
    # Stack oldest-to-newest history within each term, matching the SDK deployment.
    obs = np.concatenate([np.concatenate(h) for h in histories.values()]).reshape(1, 500)
    return np.clip(obs, -clip, clip)
