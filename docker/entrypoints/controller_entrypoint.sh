#!/bin/bash
set -e
source /opt/ros/humble/setup.bash

JOY_DEV="/dev/input/js0"

echo "[Controller Entrypoint] Watching joystick on $JOY_DEV"

start_joy() {
    echo "[Controller Entrypoint] Starting joy_node..."
    ros2 run joy joy_node --ros-args -p dev:="$JOY_DEV" &
    JOY_PID=$!
}

# Launch teleop_twist_joy immediately (stays alive)
ros2 run teleop_twist_joy teleop_node \
  --ros-args --params-file /isaac_slam/config/xbox.config.yaml -r cmd_vel:=/cmd_vel &
TELEOP_PID=$!

JOY_PID=""
JOY_PRESENT=false

while true; do
    if [ -e "$JOY_DEV" ]; then
        if [ "$JOY_PRESENT" = false ]; then
            start_joy
            JOY_PRESENT=true
        fi
        # restart joy_node if it died
        if [ -n "$JOY_PID" ] && ! kill -0 "$JOY_PID" 2>/dev/null; then
            start_joy
        fi
    else
        if [ "$JOY_PRESENT" = true ]; then
            echo "[Controller Entrypoint] Joystick removed, stopping joy_node..."
            kill "$JOY_PID" 2>/dev/null || true
            wait "$JOY_PID" 2>/dev/null || true
            JOY_PRESENT=false
        fi
    fi
    sleep 2
done
