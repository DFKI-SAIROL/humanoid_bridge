# Source this on the G1 (unitree@192.168.123.164) before running the bridge or the probe client:
#   source ~/g1_siwei/env.sh
# Same ROS setup as option 1 of the fishros prompt in ~/.bashrc, plus our private Python
# packages (numpy 1.23.5 + onnxruntime 1.16.3 in pylib/; the system numpy 1.17.4 is too old for
# onnxruntime) and our own workspace (~/g1_siwei/ws, separate from ~/sairol_ws).
G1_SIWEI="$HOME/g1_siwei"

source /opt/ros/foxy/setup.bash
source "$HOME/cyclonedds_ws/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="$HOME/cyclonedds_ws/cyclonedds.xml"
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

export PYTHONPATH="$G1_SIWEI/pylib:$PYTHONPATH"
# numpy/OpenMP thread pools would otherwise start one spinning thread per core.
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
if [ -f "$G1_SIWEI/ws/install/setup.bash" ]; then
    source "$G1_SIWEI/ws/install/setup.bash"
fi

export G1_BUNDLE="$G1_SIWEI/deploy/TI-B_inclHO_seed0"
export G1_BRIDGE_PARAMS="$G1_SIWEI/ws/src/humanoid_bridge/robot_bridge/params/G1_config.yaml"
export G1_PROBE="$G1_SIWEI/ws/src/humanoid_bridge/robot_bridge/example/G1_probe"
