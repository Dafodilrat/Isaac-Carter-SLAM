#!/bin/bash
set -e

echo "[Entrypoint] Starting container..."

# --- Load ROS 2 environment ---
if [ -f /opt/ros/humble/setup.bash ]; then
    echo "[Entrypoint] Sourcing ROS 2 Humble..."
    source /opt/ros/humble/setup.bash
else
    echo "[Entrypoint] Warning: ROS 2 Humble not found at /opt/ros/humble/setup.bash"
fi

# --- Load custom user environment (optional) ---
if [ -f /root/.bashrc ]; then
    echo "[Entrypoint] Sourcing /root/.bashrc..."
    source /root/.bashrc
fi

# --- Always build the workspace ---
echo "[Entrypoint] Building colcon workspace..."
cd /slam_ws
colcon build --symlink-install
source install/setup.bash

echo "[Entrypoint] Workspace built and sourced."

# --- Launch RViz2 + SLAM Toolbox mapping ---
echo "[Entrypoint] Launching RViz2 and SLAM Toolbox mapping..."
case "${MODE}" in
  mapping)
    echo "[Entrypoint] Running SLAM Toolbox in mapping mode..."
    exec ros2 launch isaac_slam_config mapping.launch.py
    ;;
  localization)
    echo "[Entrypoint] Running SLAM Toolbox in localization mode..."
    exec ros2 launch isaac_slam_config localization.launch.py
    ;;
  navigation)
    echo "[Entrypoint] Running Navigation2..."
    exec ros2 launch isaac_slam_config navigation.launch.py
    ;;
  *)
    echo "[Entrypoint] Unknown mode '${MODE}', defaulting to shell."
    exec /bin/bash
    ;;
esac


