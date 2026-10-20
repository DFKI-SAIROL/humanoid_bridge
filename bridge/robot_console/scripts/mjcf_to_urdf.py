#!/usr/bin/env python3
"""MJCF -> display-only URDF for the console's 3D view: kinematics and visual meshes only.

    mjcf_to_urdf.py <model.xml> <out.urdf> [--mesh-dir meshes]

Hinge joints only; collision geoms, inertials, sites and actuators are dropped.
"""

import argparse
import math
import sys
import xml.etree.ElementTree as ET


def _floats(text, default):
    return [float(v) for v in text.split()] if text else list(default)


def quat_to_rpy(quat):
    """MuJoCo quat (w x y z) -> URDF rpy (fixed-axis X, then Y, then Z)."""
    w, x, y, z = quat
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return [roll, pitch, yaw]


def _fmt(values):
    return " ".join(f"{v:.9g}" for v in values)


def _origin(parent, elem):
    pos = _floats(elem.get("pos"), (0, 0, 0))
    rpy = quat_to_rpy(_floats(elem.get("quat"), (1, 0, 0, 0)))
    ET.SubElement(parent, "origin", xyz=_fmt(pos), rpy=_fmt(rpy))


def _defaults(root):
    """class name -> {tag: attrs}, each class merged over its parent class."""
    out = {}

    def walk(node, inherited):
        name = node.get("class", "main")
        merged = {tag: dict(attrs) for tag, attrs in inherited.items()}
        for child in node:
            if child.tag != "default":
                merged.setdefault(child.tag, {}).update(child.attrib)
        out[name] = merged
        for child in node:
            if child.tag == "default":
                walk(child, merged)

    for top in root.findall("default"):
        walk(top, {})
    return out


def convert(mjcf_path, mesh_dir="meshes"):
    """Return the URDF text for the MJCF file at mjcf_path."""
    root = ET.parse(mjcf_path).getroot()
    defaults = _defaults(root)
    meshes = {m.get("name") or m.get("file").rsplit(".", 1)[0]: m.get("file")
              for m in root.iter("mesh")}
    rgba = {m.get("name"): m.get("rgba", "0.7 0.7 0.7 1") for m in root.iter("material")}

    robot = ET.Element("robot", name=root.get("model", "robot"))
    for name, color in rgba.items():
        ET.SubElement(ET.SubElement(robot, "material", name=name), "color", rgba=color)

    def attrs(elem, tag, childclass):
        cls = elem.get("class", childclass)
        merged = dict(defaults.get(cls, {}).get(tag, {}))
        merged.update(elem.attrib)
        return merged

    def add_link(body, childclass):
        link = ET.SubElement(robot, "link", name=body.get("name"))
        for geom in body.findall("geom"):
            g = attrs(geom, "geom", childclass)
            if "mesh" not in g:
                continue
            visual = ET.SubElement(link, "visual")
            _origin(visual, geom)
            ET.SubElement(ET.SubElement(visual, "geometry"), "mesh",
                          filename=f"{mesh_dir}/{meshes[g['mesh']]}")
            if g.get("material") in rgba:
                ET.SubElement(visual, "material", name=g["material"])

    def walk(body, parent, childclass):
        childclass = body.get("childclass", childclass)
        add_link(body, childclass)
        if parent is not None:
            joints = body.findall("joint")
            if len(joints) > 1:
                sys.exit(f"body {body.get('name')}: {len(joints)} joints, URDF allows one")
            j = attrs(joints[0], "joint", childclass) if joints else None
            if j is not None and j.get("type", "hinge") != "hinge":
                sys.exit(f"joint {j.get('name')}: type {j.get('type')} is not supported")
            joint = ET.SubElement(
                robot, "joint",
                name=j["name"] if j else f"{body.get('name')}_fixed",
                type="revolute" if j else "fixed",
            )
            _origin(joint, body)
            ET.SubElement(joint, "parent", link=parent)
            ET.SubElement(joint, "child", link=body.get("name"))
            if j:
                ET.SubElement(joint, "axis", xyz=_fmt(_floats(j.get("axis"), (0, 0, 1))))
                lo, hi = _floats(j.get("range"), (-math.pi, math.pi))
                ET.SubElement(joint, "limit", lower=_fmt([lo]), upper=_fmt([hi]),
                              effort="0", velocity="0")
        for child in body.findall("body"):
            walk(child, body.get("name"), childclass)

    for top in root.find("worldbody").findall("body"):
        walk(top, None, "main")
    ET.indent(robot)
    return ET.tostring(robot, encoding="unicode") + "\n"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("mjcf")
    parser.add_argument("urdf")
    parser.add_argument("--mesh-dir", default="meshes", help="mesh path prefix in the URDF")
    args = parser.parse_args(argv)
    text = convert(args.mjcf, args.mesh_dir)
    with open(args.urdf, "w", encoding="utf-8") as out:
        out.write(text)


if __name__ == "__main__":
    main()
