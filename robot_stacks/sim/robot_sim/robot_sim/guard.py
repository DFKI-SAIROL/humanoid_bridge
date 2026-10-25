"""The sim talks DDS over loopback only: on a robot NIC, the bridge reads simulated state."""

import re
from pathlib import Path

LOOPBACK = {"lo", "127.0.0.1", "localhost"}
FIX = "source bridge/robot_bridge/tests/integration/loopback_dds.sh first"


def cyclone_interfaces(uri):
    """Interfaces a CYCLONEDDS_URI binds, one entry per element; None for an unnamed one."""
    text = uri.strip()
    if text.startswith("file://"):
        text = Path(text[len("file://") :]).read_text()
    elif not text.startswith("<"):
        text = Path(text).read_text()
    found = []
    for attrs in re.findall(r"<NetworkInterface\b([^>]*)>", text):
        named = re.search(r'\b(?:name|address)="([^"]*)"', attrs)
        found.append(named.group(1) if named else None)
    found += re.findall(r"<NetworkInterfaceAddress>\s*([^<\s]+)\s*</NetworkInterfaceAddress>", text)
    return found


def not_loopback(environ):
    """None when DDS stays on loopback; else the reason it may not."""
    rmw = environ.get("RMW_IMPLEMENTATION") or "rmw_fastrtps_cpp"
    if rmw == "rmw_cyclonedds_cpp":
        uri = environ.get("CYCLONEDDS_URI", "")
        if not uri.strip():
            return "CYCLONEDDS_URI is unset: CycloneDDS picks a NIC itself"
        try:
            found = cyclone_interfaces(uri)
        except OSError as exc:
            return f"CYCLONEDDS_URI cannot be read: {exc}"
        if not found or any(name not in LOOPBACK for name in found):
            return f"CYCLONEDDS_URI binds {found or 'no named interface'}, not lo only"
        return None
    if environ.get("ROS_LOCALHOST_ONLY") == "1":
        return None
    if environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE") == "LOCALHOST":
        return None
    return f"{rmw} without ROS_LOCALHOST_ONLY=1"
