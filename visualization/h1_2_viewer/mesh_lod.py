"""Display-only binary STL decimation. Originals and physical geometry stay intact."""
import struct
import numpy as np

_STL = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])


def simplify_stl(source, destination, face_limit):
    """Return (input faces, output faces); unsupported formats keep their original file."""
    if source.suffix.lower() != '.stl' or not face_limit:
        return None
    raw = source.read_bytes()
    if len(raw) < 84:
        return None
    count = struct.unpack_from('<I', raw, 80)[0]
    if len(raw) != 84 + count * 50:
        return None  # ASCII STL or non-STL: let the official loader handle it.
    if count <= face_limit:
        return count, count
    from fast_simplification import simplify
    triangles = np.frombuffer(raw, dtype=_STL, offset=84)['vertices']
    vertices, inverse = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    if not np.all(np.isfinite(vertices)):
        raise ValueError(f'Nonfinite STL vertices: {source}')
    points, faces = simplify(vertices.astype(np.float64), inverse.reshape(-1, 3), target_count=face_limit)
    if len(faces) == 0 or not np.all(np.isfinite(points)):
        raise ValueError(f'Mesh simplification produced invalid geometry: {source}')
    result = np.zeros(len(faces), dtype=_STL)
    result['vertices'] = points[faces]
    v = result['vertices']
    normals = np.cross(v[:, 1] - v[:, 0], v[:, 2] - v[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    result['normal'] = normals / np.maximum(lengths[:, None], 1e-20)
    destination.write_bytes(b'H1-2 temporary display mesh'.ljust(80, b'\0') + struct.pack('<I', len(faces)) + result.tobytes())
    return count, len(faces)
