#!/bin/bash

# --- Experiment 7! Chat Interface Wrapper ---

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$( dirname "$SCRIPT_DIR" )"

# 1. Load Environment Variables from project root
if [ -f "$PROJECT_ROOT/.env" ]; then
    set -a
    source "$PROJECT_ROOT/.env"
    set +a
fi

# 2. Setup ROS 2 Environment if needed (only if rclpy isn't found or for bob_llm mode)
# In many cases, it's already sourced in the shell or .bashrc
if [ -f "/opt/ros/humble/setup.bash" ] && [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/humble/setup.bash
fi

# 3. Handle Local Workspace sourcing (if available)
if [ -f "$PROJECT_ROOT/ros2_ws/install/setup.bash" ]; then
    source "$PROJECT_ROOT/ros2_ws/install/setup.bash"
fi

# 4. Invoke the Python Chat Client
python3 "$SCRIPT_DIR/core/chat_client.py" "$@"
