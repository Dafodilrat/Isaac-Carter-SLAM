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

echo "[Entrypoint] Launching joystick, teleop, and RViz2..."

# Start joy_node (reads Xbox controller)
ros2 run joy joy_node --ros-args -p dev:="/dev/input/js0" &
JOY_PID=$!

# Start teleop_twist_joy (maps joystick -> /cmd_vel)
ros2 run teleop_twist_joy teleop_node \
  --ros-args --params-file /xbox.config.yaml -r cmd_vel:=/cmd_vel &
TELEOP_PID=$!

# Launch RViz2 in the foreground (this will block)
ros2 run rviz2 rviz2 "$@"
RVIZ_STATUS=$?

echo "[Entrypoint] RViz closed. Shutting down teleop and joystick nodes..."

# Kill background nodes
kill $JOY_PID $TELEOP_PID 2>/dev/null || true
wait $JOY_PID $TELEOP_PID 2>/dev/null || true

echo "[Entrypoint] Clean exit."
exit $RVIZ_STATUS
