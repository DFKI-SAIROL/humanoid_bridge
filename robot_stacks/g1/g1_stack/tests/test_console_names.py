"""The console's config-to-URDF naming rule against the names gear/goal.py spells out."""

from pathlib import Path

from robot_console.catalog import BODY, load_catalog

from g1_stack.gear.goal import UPPER_BODY_JOINTS

G1_CONFIG = Path(__file__).resolve().parents[4] / "bridge/robot_bridge/params/G1_config.yaml"


def test_the_naming_rule_reproduces_every_upper_body_joint():
    catalog = load_catalog(G1_CONFIG, hands=True)
    for urdf_joint in UPPER_BODY_JOINTS:
        assert catalog.by_urdf(urdf_joint).urdf_joint == urdf_joint


def test_leg_names_are_not_goal_names():
    legs = {entry.urdf_joint for entry in load_catalog(G1_CONFIG).group(BODY)[:12]}
    assert not legs & set(UPPER_BODY_JOINTS)
