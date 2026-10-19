"""The motor table, against the shipped G1_config.yaml."""

from __future__ import annotations

import dataclasses
import json

import pytest

from g1_stack.gear.goal import UPPER_BODY_JOINTS
from g1_stack.motor_probe import catalog as catalog_mod
from g1_stack.motor_probe.catalog import BODY, LEFT_HAND, RIGHT_HAND, MotorEntry, load_catalog

LEG_URDF_JOINTS = (
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
)


@pytest.fixture(scope="module")
def catalog():
    return load_catalog()


def test_the_default_config_path_is_the_shipped_yaml():
    path = catalog_mod.default_config_path()
    assert path.is_file()
    assert path.name == "G1_config.yaml"


def test_forty_three_motors_in_group_order(catalog):
    assert len(catalog.entries) == 43
    groups = [entry.group for entry in catalog.entries]
    assert groups == [BODY] * 29 + [LEFT_HAND] * 7 + [RIGHT_HAND] * 7


def test_every_group_covers_its_indices_exactly_once(catalog):
    for group, size in ((BODY, 29), (LEFT_HAND, 7), (RIGHT_HAND, 7)):
        entries = catalog.group(group)
        assert [entry.index for entry in entries] == list(range(size))


def test_body_indices_follow_the_joint_names_order(catalog):
    body = catalog.group(BODY)
    assert body[0].name == "L_LEG_HIP_PITCH"
    assert body[12].name == "WAIST_YAW"
    assert body[28].name == "R_WRIST_YAW"


def test_urdf_names_are_unique_across_all_motors(catalog):
    names = [entry.urdf_joint for entry in catalog.entries]
    assert len(set(names)) == 43


@pytest.mark.parametrize(
    "name,index,q_min,q_max,dq_limit,tau_limit,kp,kd",
    [
        ("L_ELBOW", 18, -1.0472, 2.0944, 37.0, 25.0, 50.0, 2.0),
        ("L_LEG_KNEE", 3, -0.087267, 2.8798, 20.0, 139.0, 150.0, 4.0),
        ("WAIST_YAW", 12, -2.618, 2.618, 32.0, 88.0, 300.0, 3.0),
        ("R_WRIST_YAW", 28, -1.614429558, 1.614429558, 22.0, 5.0, 20.0, 1.0),
        ("R_HAND_THUMB_0", 0, -1.04719755, 1.04719755, 6.857, 2.45, 2.0, 0.5),
        ("L_HAND_INDEX_1", 6, -1.74532925, 0.0, 12.0, 1.4, 1.0, 0.2),
    ],
)
def test_values_are_transcribed_from_the_yaml(
    catalog, name, index, q_min, q_max, dq_limit, tau_limit, kp, kd
):
    entry = catalog.by_name(name)
    assert entry.index == index
    assert (entry.q_min, entry.q_max) == (q_min, q_max)
    assert (entry.dq_limit, entry.tau_limit) == (dq_limit, tau_limit)
    assert (entry.kp, entry.kd) == (kp, kd)


def test_the_two_hands_are_mirrored_not_shared(catalog):
    """One shared table per hand would be wrong: the ranges differ by side."""
    left = catalog.by_name("L_HAND_MIDDLE_0")
    right = catalog.by_name("R_HAND_MIDDLE_0")
    assert (left.q_min, left.q_max) == (-1.57079632, 0.0)
    assert (right.q_min, right.q_max) == (0.0, 1.57079632)

    mirrored = [
        (left_entry.name, right_entry.name)
        for left_entry, right_entry in zip(catalog.group(LEFT_HAND), catalog.group(RIGHT_HAND))
        if (left_entry.q_min, left_entry.q_max) != (right_entry.q_min, right_entry.q_max)
    ]
    assert len(mirrored) == 6  # every hand joint but THUMB_0, which is symmetric


def test_the_naming_rule_reproduces_every_upper_body_joint(catalog):
    """The rule that derives the leg names is only trustworthy if it also
    reproduces the names gear/goal.py already spells out."""
    for urdf_joint in UPPER_BODY_JOINTS:
        assert catalog.by_urdf(urdf_joint).urdf_joint == urdf_joint


@pytest.mark.parametrize(
    "urdf_joint,name",
    [
        ("left_elbow_joint", "L_ELBOW"),
        ("waist_pitch_joint", "WAIST_PITCH"),
        ("right_shoulder_roll_joint", "R_SHOULDER_ROLL"),
        ("left_hand_thumb_0_joint", "L_HAND_THUMB_0"),
        ("right_hand_index_1_joint", "R_HAND_INDEX_1"),
        ("left_knee_joint", "L_LEG_KNEE"),
        ("right_ankle_roll_joint", "R_LEG_ANKLE_ROLL"),
    ],
)
def test_by_urdf_resolves_to_the_config_name(catalog, urdf_joint, name):
    assert catalog.by_urdf(urdf_joint).name == name


def test_the_twelve_leg_joints_get_derived_urdf_names(catalog):
    derived = [entry.urdf_joint for entry in catalog.group(BODY)[:12]]
    assert derived == list(LEG_URDF_JOINTS)
    assert not set(LEG_URDF_JOINTS) & set(UPPER_BODY_JOINTS)


def test_unknown_lookups_raise(catalog):
    with pytest.raises(KeyError):
        catalog.by_name("L_TAIL")
    with pytest.raises(KeyError):
        catalog.by_urdf("left_tail_joint")
    with pytest.raises(KeyError):
        catalog.group("legs")


def test_to_json_round_trips(catalog):
    rows = catalog.to_json()
    assert len(rows) == 43
    assert set(rows[0]) == {field.name for field in dataclasses.fields(MotorEntry)}
    assert json.loads(json.dumps(rows)) == rows
