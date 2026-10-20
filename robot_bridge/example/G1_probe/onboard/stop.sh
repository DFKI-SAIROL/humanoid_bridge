#!/bin/bash
# Stop the probe client, then the bridge.
# The client treats the first SIGINT like 'q': damps if the policy is running (1 s ramp), calls
# /stop_control, saves the current run and session, exits. Only if it is still up after 10 s
# does it get a second signal (immediate exit).
wait_gone() {  # pattern, seconds
    for _ in $(seq $((4 * $2))); do
        pgrep -f "$1" >/dev/null || return 0
        sleep 0.25
    done
    return 1
}

pkill -INT -f "python3 -u G1_probe.py --bundle" || true
if ! wait_gone "python3 -u G1_probe.py --bundle" 10; then
    echo "client still running after 10 s, interrupting again"
    pkill -INT -f "python3 -u G1_probe.py --bundle" || true
    wait_gone "python3 -u G1_probe.py --bundle" 5 || pkill -KILL -f "python3 -u G1_probe.py --bundle"
fi

pkill -INT -f "lib/robot_bridge/G1_bridge" || true
wait_gone "G1_bridge --ros-args" 5 || pkill -TERM -f "G1_bridge --ros-args"
pgrep -fa "G1_probe.py --bundle|G1_bridge --ros-args" || echo "stopped"
