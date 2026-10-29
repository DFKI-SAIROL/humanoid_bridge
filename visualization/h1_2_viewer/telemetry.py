"""Transport-independent state conversion. Output order is defined by the YAML profile."""
import numpy as np


def unitree_body(message, joints):
    required = max(j.motor_index for j in joints)
    if len(message.motor_state) <= required:
        raise ValueError(f'Motor array is too short: need index {required}')
    q, dq, tau = [], [], []
    for joint in joints:
        motor = message.motor_state[joint.motor_index]
        q.append(motor.q * joint.scale + joint.offset)
        dq.append(motor.dq * joint.scale)
        # Configuration scale transforms model coordinates; torque stays raw Nm.
        tau.append(motor.tau_est)
    values = np.asarray([q, dq, tau], dtype=float)
    if not np.all(np.isfinite(values)):
        raise ValueError('Non-finite motor feedback')
    return values


def named_body(message, joints):
    if len(set(message.name)) != len(message.name):
        raise ValueError('Duplicate JointState names')
    lookup = {name: i for i, name in enumerate(message.name)}
    # Named JointState is already in model coordinates: no hardware scale/offset.
    rows = []
    for field in ('position', 'velocity', 'effort'):
        values = getattr(message, field)
        if field != 'position' and not len(values):
            rows.append([float('nan')] * len(joints))
            continue
        if len(values) != len(message.name):
            raise ValueError(f'JointState.{field} must contain a value for every named joint')
        try:
            rows.append([values[lookup[j.name]] for j in joints])
        except KeyError as exc:
            raise ValueError(f'Missing mapped JointState name: {exc}') from exc
    values = np.asarray(rows, dtype=float)
    if not np.all(np.isfinite(values[0])) or np.any(np.isinf(values[1:])):
        raise ValueError('Invalid named joint feedback')
    return values
