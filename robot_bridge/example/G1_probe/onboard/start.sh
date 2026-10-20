#!/bin/bash
# Start the G1 bridge and the probe client (with web commander) on the robot, detached from this
# shell: they keep running if the SSH session or the ethernet cable drops.
#   ~/g1_siwei/start.sh [extra G1_probe.py args, e.g. --allow-untrained]
# Stop with ~/g1_siwei/stop.sh. Logs: ~/g1_siwei/logs/{bridge,client}.log, runs in ~/g1_siwei/probe_logs/.
set -e
source "$HOME/g1_siwei/env.sh"
LOGS="$G1_SIWEI/logs"
mkdir -p "$LOGS"

if pgrep -f "G1_bridge --ros-args" >/dev/null || pgrep -f "G1_probe.py --bundle" >/dev/null; then
    echo "bridge or client already running; run ~/g1_siwei/stop.sh first"
    exit 1
fi

# The exit code goes into the log (245 = 256 - 11: segfault).
setsid nohup bash -c 'ros2 run robot_bridge G1_bridge --ros-args --params-file "$G1_BRIDGE_PARAMS"; echo "G1_bridge exited with code $?"' \
    > "$LOGS/bridge.log" 2>&1 < /dev/null &
sleep 2
cd "$G1_PROBE"
setsid nohup python3 -u G1_probe.py --bundle "$G1_BUNDLE" --web-host 0.0.0.0 --web-port 8080 \
    --log-dir "$G1_SIWEI/probe_logs" "$@" > "$LOGS/client.log" 2>&1 < /dev/null &
sleep 3

pgrep -f "G1_bridge --ros-args" >/dev/null && echo "bridge running" || { echo "bridge FAILED, see $LOGS/bridge.log"; exit 1; }
pgrep -f "G1_probe.py --bundle" >/dev/null && echo "client running" || { echo "client FAILED, see $LOGS/client.log"; exit 1; }
echo "Web commander: http://192.168.123.164:8080/ (cable) or http://10.42.0.1:8080/ (robot hotspot)"
echo "If the bridge waits for debug mode (tail $LOGS/bridge.log): press L2+R2 on the remote."
