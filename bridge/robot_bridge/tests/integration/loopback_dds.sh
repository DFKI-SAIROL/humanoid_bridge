# shellcheck shell=bash
# Sourced by every run_fake_*_test.sh: discovery on loopback only, so a fake run never reaches
# a cabled robot, even from a shell that sourced scripts/setup_unitree.sh. Same pinning as tools/test.sh.
if [ "${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" = rmw_cyclonedds_cpp ]; then
    export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" multicast="true"/></Interfaces></General></Domain></CycloneDDS>'
else
    export ROS_LOCALHOST_ONLY=1
    unset CYCLONEDDS_URI
fi
