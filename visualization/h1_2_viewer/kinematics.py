"""URDF adapter shared by measured-state TF, trajectory preview and demo.

Computes geometry only. Never modifies measured state or executes a trajectory.
"""
from .spatial_data import Pose
import numpy as np


class UrdfKinematics:
    def __init__(self, model):
        self.model = model
        self.root = 'h1_2/' + model.tree.root_link().name
        self.aliases = {name: 'h1_2/' + name for name in model.links}
        self.by_child = {j.find('child').get('link'): j for j in model.joints.values()}

    def values(self, values):
        values = dict(values)
        pending = {name: j.find('mimic') for name, j in self.model.joints.items()
                   if j.find('mimic') is not None and name not in values}
        while pending:
            ready = [name for name, m in pending.items() if m.get('joint') in values]
            if not ready:
                break
            for name in ready:
                m = pending.pop(name)
                values[name] = values[m.get('joint')] * float(m.get('multiplier', '1')) + float(m.get('offset', '0'))
        return values

    def local(self, name, value):
        t = self.model.tree.get_joint_by_name(name).compute_transform(float(value), clamp=False)
        return (t.child_frame.as_arrow_array().to_pylist()[0],
                Pose(t.parent_frame.as_arrow_array().to_pylist()[0],
                     t.translation.as_arrow_array().to_pylist()[0],
                     t.quaternion.as_arrow_array().to_pylist()[0]))

    def tip_matrix(self, values, tip_link, *, require_complete=False):
        tip_link = tip_link.removeprefix('h1_2/')
        if tip_link not in self.model.links:
            raise ValueError(f'Unknown trajectory tip_link: {tip_link}')
        values = self.values(values)
        chain, child = [], tip_link
        while child in self.by_child:
            joint = self.by_child[child]
            chain.append(joint)
            child = joint.find('parent').get('link')
        matrix = np.eye(4)
        for joint in reversed(chain):
            name = joint.get('name')
            if require_complete and joint.get('type') != 'fixed' and name not in values:
                raise ValueError(f'Trajectory needs measured or planned state for {name}')
            _, pose = self.local(name, values.get(name, 0))
            matrix = matrix @ pose.matrix()
        return matrix
