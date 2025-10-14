#!/bin/bash
set -e

# Explicitly source .bashrc to pull in ROS, venv, and Isaac env vars
if [ -f /root/.bashrc ]; then
    echo "[Entrypoint] Sourcing /root/.bashrc..."
    source /root/.bashrc
fi

# Default command: Isaac Sim
ISAACSIM_CMD=${ISAACSIM_CMD:-/isaac-sim/isaac-sim.sh}

echo "[Entrypoint] Launching Isaac Sim with command: $ISAACSIM_CMD $@"
exec $ISAACSIM_CMD "$@"
