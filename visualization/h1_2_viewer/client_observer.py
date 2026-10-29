"""Read-only tap on existing client command topics; never imports the controller."""
import numpy as np


def body_target(message, joints):
    required = max(j.motor_index for j in joints)
    if len(message.motor_cmd) <= required:
        raise ValueError('Client command motor array is too short')
    q = np.asarray([message.motor_cmd[j.motor_index].q * j.scale + j.offset for j in joints], dtype=float)
    if not np.all(np.isfinite(q)):
        raise ValueError('Nonfinite client target positions')
    return q


def attach(node, profile):
    source = profile.settings['source']
    if source['body_target_topic']:
        from bridge_interface.msg import RobotCmd
        node.subscribe(source['body_target_topic'], RobotCmd, 'body_target')
    if source['hand_target_topic']:
        from unitree_go.msg import MotorCmds
        node.subscribe(source['hand_target_topic'], MotorCmds, 'hand_target')


def drain(node, dashboard, profile):
    # Called after feedback and TF. Latest-only mailbox bounds observer work at viewer Hz.
    for stream, message in getattr(node, 'client_pending', {}).items():
        try:
            if stream == 'body_target':
                dashboard.body_target(body_target(message, profile.joints))
            else:
                dashboard.hand_target([motor.q for motor in message.cmds])
        except (ValueError, TypeError) as exc:
            dashboard.errors[stream] = str(exc)
    node.client_pending = {}
