#!/bin/bash
set -e

# Load ROS environment
if [ -f /opt/ros/humble/setup.bash ]; then
    echo "[Entrypoint] Sourcing ROS 2 Humble..."
    source /opt/ros/humble/setup.bash
fi

# Source optional custom env
if [ -f /root/.bashrc ]; then
    source /root/.bashrc
fi

echo "[Entrypoint] Launching rviz2..."
exec ros2 run rviz2 rviz2 "$@"
