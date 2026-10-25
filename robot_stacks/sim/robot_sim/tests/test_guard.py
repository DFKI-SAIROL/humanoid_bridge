"""The loopback guard: no ROS, no mujoco."""

import pathlib
import re

import pytest

from robot_sim.guard import cyclone_interfaces, not_loopback

REPO = pathlib.Path(__file__).resolve().parents[4]
WRAP = "<CycloneDDS><Domain><General>{}</General></Domain></CycloneDDS>"
AUTO = '<NetworkInterface autodetermine="true"/>'


def uri(*interfaces, extra=""):
    elements = "".join(f'<NetworkInterface name="{i}" multicast="true"/>' for i in interfaces)
    return WRAP.format(f"<Interfaces>{elements}{extra}</Interfaces>")


def cyclone(text):
    return {"RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp", "CYCLONEDDS_URI": text}


def script_uri(path, iface=None):
    """The URI a repo script exports, with its $BRIDGE_IFACE filled in."""
    text = (REPO / path).read_text()
    found = re.search(r"CYCLONEDDS_URI='(.*?)'(?:\"\$BRIDGE_IFACE\"'(.*?)')?", text, re.S)
    return found.group(1) + (iface + found.group(2) if found.group(2) else "")


def test_loopback_dds_sh_passes():
    loopback = script_uri("bridge/robot_bridge/tests/integration/loopback_dds.sh")
    assert not_loopback(cyclone(loopback)) is None


def test_setup_unitree_sh_is_refused():
    robot = script_uri("scripts/setup_unitree.sh", "enp4s0")
    assert "enp4s0" in not_loopback(cyclone(robot))


@pytest.mark.parametrize(
    "environ",
    [
        {"RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp"},
        cyclone(uri("lo", "enp4s0")),
        cyclone(uri()),
        cyclone(uri("lo", extra=AUTO)),
        {"RMW_IMPLEMENTATION": "rmw_fastrtps_cpp"},
        {},
    ],
    ids=["unset", "lo-and-nic", "none-named", "lo-and-auto", "fastrtps", "default-rmw"],
)
def test_anything_beyond_loopback_is_refused(environ):
    assert not_loopback(environ)


def test_fastrtps_localhost_only_passes():
    environ = {"RMW_IMPLEMENTATION": "rmw_fastrtps_cpp", "ROS_LOCALHOST_ONLY": "1"}
    assert not_loopback(environ) is None


def test_legacy_address_form_is_read():
    legacy = "<NetworkInterfaceAddress>enp4s0</NetworkInterfaceAddress>"
    assert cyclone_interfaces(WRAP.format(legacy)) == ["enp4s0"]


def test_file_uri_is_read(tmp_path):
    path = tmp_path / "cyclone.xml"
    path.write_text(uri("lo"))
    assert not_loopback(cyclone(f"file://{path}")) is None


def test_missing_file_is_refused(tmp_path):
    assert "cannot be read" in not_loopback(cyclone(f"file://{tmp_path}/none.xml"))
