if [ "$USER" != "unitree" ]; then
    UNITREE_NETWORK_INTERFACE="${UNITREE_NETWORK_INTERFACE:-enp0s31f6}"
    if [[ ! "$UNITREE_NETWORK_INTERFACE" =~ ^[a-zA-Z0-9_.:-]+$ ]]; then
        echo "Invalid UNITREE_NETWORK_INTERFACE: $UNITREE_NETWORK_INTERFACE" >&2
        return 1
    fi
    echo "Set CycloneDDS and network interface $UNITREE_NETWORK_INTERFACE; ROS environment must already be activated"
    #source /opt/ros/humble/setup.bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces>
                                <NetworkInterface name=\"$UNITREE_NETWORK_INTERFACE\" priority=\"default\" multicast=\"default\" />
                            </Interfaces></General></Domain></CycloneDDS>"
fi

if [ -z "$ROS_DISTRO" ]; then
    echo "ROS_DISTRO is not set. Please source ros2"
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

source "$WORKSPACE_DIR/install/local_setup.bash" || return 1
echo "Sourced workspace: $WORKSPACE_DIR"
