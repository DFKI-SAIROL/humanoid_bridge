"""URDF-to-Rerun adapter using Rerun's official UrdfTree implementation.

Model edits remain in their source folder. URI resolution and display tint are
applied to an ephemeral URDF; neither upstream nor custom source files are edited.
"""
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
import rerun as rr
from rerun.urdf import UrdfTree


class RobotModel:
    def __init__(self, profile):
        self.profile = profile
        self.urdf = profile.urdf
        if not self.urdf.is_file():
            raise FileNotFoundError(f'{self.urdf}\nRun h1-2-models fetch, or supply --config /path/to/viewer.yaml / --urdf /path/to/model.urdf')
        xml = ET.parse(self.urdf).getroot()
        if xml.tag != 'robot':
            raise ValueError('Viewer expects a URDF <robot>. MuJoCo .xml edits belong to simulation; select/edit the corresponding URDF for this viewer.')
        self.joints = {j.get('name'): j for j in xml.findall('joint')}
        self.links = {link.get('name'): link for link in xml.findall('link')}
        missing = {j.name for j in profile.joints} - self.joints.keys()
        if missing:
            raise ValueError(f'Joint mapping references names missing from URDF: {sorted(missing)}')
        self._temp = tempfile.TemporaryDirectory(prefix="h1_2_viewer_urdf_")
        self.mesh_stats = [0, 0]
        self.meshes = []
        self._logged_joint_values = {}
        for tag, attribute in (('mesh', 'filename'), ('texture', 'filename')):
            for item in xml.findall(f'.//{tag}'):
                path = self.resolve_resource(item.get(attribute))
                item.set(attribute, str(path))
                if tag == 'mesh':
                    self.meshes.append(path)
        # Only visual resources are simplified; collision/source URDF are untouched.
        from .mesh_lod import simplify_stl
        lod_paths = {}
        for item in xml.findall('.//visual/geometry/mesh'):
            source = Path(item.get('filename'))
            if source not in lod_paths:
                destination = Path(self._temp.name) / f'mesh_{len(lod_paths)}.stl'
                stats = simplify_stl(source, destination, profile.settings['model']['mesh_face_limit'])
                if stats:
                    self.mesh_stats = [a + b for a, b in zip(self.mesh_stats, stats)]
                lod_paths[source] = destination if destination.is_file() else source
            item.set('filename', str(lod_paths[source]))
        # The official loader logs collision assets as well as visuals. Do not send
        # hidden full-resolution copies to a browser; originals stay on disk.
        if not profile.settings['model']['show_collisions']:
            for link in xml.findall('link'):
                for collision in list(link.findall('collision')):
                    link.remove(collision)
        tint = profile.settings['model']['display_tint']
        if tint is not None:
            for visual in xml.findall('.//visual'):
                material = visual.find('material')
                if material is None:
                    material = ET.SubElement(visual, 'material')
                # Inline tint avoids dependence on named-material lookup and improves contrast.
                material.attrib.clear()
                material.set('name', 'viewer_display_tint')
                for child in list(material):
                    material.remove(child)
                ET.SubElement(material, 'color', rgba=' '.join(str(v) for v in tint))
        resolved = Path(self._temp.name) / 'resolved.urdf'
        ET.ElementTree(xml).write(resolved, encoding='utf-8', xml_declaration=True)
        self.tree = UrdfTree.from_file_path(resolved, entity_path_prefix='robot',
                                           frame_prefix='h1_2/', static_transform_entity_path='robot/tf_static')
        from .kinematics import UrdfKinematics
        self.kinematics = UrdfKinematics(self)
        self.hand_reference_note = 'Unmapped joints (including fingers) retain the URDF reference pose.'

    def resolve_resource(self, uri):
        if not uri:
            raise ValueError('Empty URDF resource URI')
        if uri.startswith('package://'):
            package, _, relative = uri[len('package://'):].partition('/')
            root = self.profile.package_paths.get(package)
            if root is None:
                # Conventional description package root containing this URDF.
                root = next((p for p in self.urdf.parents if p.name == package), None)
            if root is None:
                raise ValueError(f'Unresolved package://{package}; set model.package_paths.{package} in YAML')
            path = root / relative
        elif uri.startswith('file://'):
            from urllib.parse import unquote, urlparse
            path = Path(unquote(urlparse(uri).path))
        elif '://' in uri:
            raise ValueError(f'Only local model resources are supported, got {uri}')
        else:
            path = self.urdf.parent / uri
        path = path.resolve()
        if not path.is_file():
            raise FileNotFoundError(path)
        return path

    def log_geometry(self):
        rr.log('robot', rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
        rr.log('robot', rr.CoordinateFrame(frame='h1_2/' + self.tree.root_link().name), static=True)
        self.tree.log_urdf_to_recording()

    def log_body(self, positions):
        q = np.asarray(positions, dtype=float)
        if q.shape != (len(self.profile.joints),) or not np.all(np.isfinite(q)):
            raise ValueError('Body state must match the configured joint count and be finite')
        values = self.kinematics.values({joint.name: float(value) for joint, value in zip(self.profile.joints, q)})
        for name, value in values.items():
            previous = self._logged_joint_values.get(name)
            if previous is not None and abs(previous - value) <= self.profile.settings['model']['joint_epsilon_rad']:
                continue
            self._logged_joint_values[name] = value
            joint = self.tree.get_joint_by_name(name)
            rr.log(f'robot/transforms/{name}', joint.compute_transform(value, clamp=False))

    def close(self):
        self._temp.cleanup()
