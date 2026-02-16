#!/bin/bash
set -e

echo "[*] NEXUS Entity Boot: $NAME ($ENTITY_CATEGORY - $WORKSPACE_POLICY)"

# 1. Check for Auto-Onboarding
if [ ! -f /app/ros2_ws_local/install/setup.bash ] && [ "$WORKSPACE_POLICY" = "isolated" ]; then
    echo "[*] Isolated Workspace detected. Running internal onboarding..."
    /app/onboarding.sh
fi

# 1.1 Fast Python Dependency Check
if [ -d "/app/ros2_ws_local/src" ]; then
    echo "[*] Ensuring Python dependencies are met..."
    find "/app/ros2_ws_local/src" -maxdepth 2 -name "requirements.txt" -exec pip install --no-cache-dir -r {} \;
fi

# 2. Source ROS Environments
source /opt/ros/humble/setup.bash

# Global Underlay
if [ -f /app/ros2_ws/install/setup.bash ]; then 
    echo "[*] Sourcing Global Workspace..."
    source /app/ros2_ws/install/setup.bash
fi

# Local Overlay
if [ -f /app/ros2_ws_local/install/setup.bash ]; then
    echo "[*] Sourcing Local Overlay..."
    source /app/ros2_ws_local/install/setup.bash
fi

# 3. Launch
echo "[*] Launching ROS 2 Generic System..."
exec ros2 launch bob_launch generic.launch.py
