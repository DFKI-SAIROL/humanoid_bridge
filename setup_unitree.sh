if [ "$USER" != "unitree" ]; then
    echo "Set CycloneDDS and network interface enp2s0; ROS environment must already be activated"
    #source /opt/ros/humble/setup.bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                                <NetworkInterface name="enp2s0" priority="default" multicast="default" />
                            </Interfaces></General></Domain></CycloneDDS>'
fi

if [ -z "$ROS_DISTRO" ]; then
    echo "ROS_DISTRO is not set. Please source ros2"
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR=$(dirname $(dirname $SCRIPT_DIR))

source "$WORKSPACE_DIR/install/local_setup.bash" || return 1
echo "Sourced workspace: $WORKSPACE_DIR"
