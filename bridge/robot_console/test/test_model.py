"""The 3D model: MJCF -> URDF conversion and the SOURCE tag the page shows."""

from __future__ import annotations

import importlib.util
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from robot_console.service import model_source

_SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "mjcf_to_urdf.py"
_spec = importlib.util.spec_from_file_location("mjcf_to_urdf", _SCRIPT)
mjcf_to_urdf = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(mjcf_to_urdf)

MJCF = """
<mujoco model="toy">
  <default>
    <default class="toy">
      <default class="visual"><geom type="mesh" material="silver" contype="0"/></default>
      <default class="collision"><geom type="capsule"/></default>
    </default>
  </default>
  <asset>
    <material name="silver" rgba="0.7 0.7 0.7 1"/>
    <material name="black" rgba="0.2 0.2 0.2 1"/>
    <mesh name="base" file="base.STL"/>
    <mesh file="arm.STL"/>
  </asset>
  <worldbody>
    <body name="base" pos="0 0 0.8" childclass="toy">
      <freejoint name="floating"/>
      <geom class="visual" mesh="base"/>
      <geom class="collision" size="0.1" fromto="0 0 0 0 0 -0.1"/>
      <body name="arm" pos="0.1 0.2 0.3" quat="0.7071068 0 0 0.7071068">
        <joint name="arm_joint" axis="0 1 0" range="-1 2"/>
        <geom class="visual" material="black" mesh="arm" pos="0 0 0.05"/>
        <body name="tip" pos="0 0 0.4"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""


@pytest.fixture()
def urdf(tmp_path):
    path = tmp_path / "toy.xml"
    path.write_text(MJCF)
    return ET.fromstring(mjcf_to_urdf.convert(path))


def test_quat_to_rpy_matches_known_rotations():
    assert mjcf_to_urdf.quat_to_rpy([1, 0, 0, 0]) == pytest.approx([0, 0, 0])
    half = math.sqrt(0.5)
    assert mjcf_to_urdf.quat_to_rpy([half, 0, 0, half]) == pytest.approx([0, 0, math.pi / 2])
    assert mjcf_to_urdf.quat_to_rpy([half, half, 0, 0]) == pytest.approx([math.pi / 2, 0, 0])


def test_each_body_becomes_a_link_and_the_free_root_has_no_joint(urdf):
    assert [link.get("name") for link in urdf.iter("link")] == ["base", "arm", "tip"]
    assert [j.get("name") for j in urdf.iter("joint")] == ["arm_joint", "tip_fixed"]


def test_a_hinge_keeps_origin_axis_and_range(urdf):
    joint = urdf.find("joint[@name='arm_joint']")
    assert joint.get("type") == "revolute"
    assert joint.find("parent").get("link") == "base"
    assert joint.find("origin").get("xyz") == "0.1 0.2 0.3"
    rpy = [float(v) for v in joint.find("origin").get("rpy").split()]
    assert rpy == pytest.approx([0, 0, math.pi / 2])
    assert joint.find("axis").get("xyz") == "0 1 0"
    assert (joint.find("limit").get("lower"), joint.find("limit").get("upper")) == ("-1", "2")


def test_a_body_without_a_joint_is_fixed(urdf):
    assert urdf.find("joint[@name='tip_fixed']").get("type") == "fixed"


def test_visuals_keep_mesh_pose_and_material_and_collisions_are_dropped(urdf):
    base = urdf.find("link[@name='base']")
    assert len(base.findall("visual")) == 1  # the capsule is not a visual
    assert base.find("visual/geometry/mesh").get("filename") == "meshes/base.STL"
    assert base.find("visual/material").get("name") == "silver"  # from the default class
    arm = urdf.find("link[@name='arm']/visual")
    assert arm.find("geometry/mesh").get("filename") == "meshes/arm.STL"  # unnamed mesh
    assert arm.find("material").get("name") == "black"
    assert arm.find("origin").get("xyz") == "0 0 0.05"


def test_model_source_reads_the_first_line_of_source(tmp_path):
    (tmp_path / "SOURCE").write_text("mjlab @ 6fc2a60 g1.xml\nmore\n")
    assert model_source(tmp_path / "g1.urdf") == "mjlab @ 6fc2a60 g1.xml"


def test_model_source_without_a_file_says_unknown(tmp_path):
    assert model_source(tmp_path / "g1.urdf").startswith("source unknown")
    assert model_source(None) is None
