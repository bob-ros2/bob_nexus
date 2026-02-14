#!/bin/bash
set -e

# --- Experiment 7! Onboarding Script ---
# This script initializes the bob_nexus environment inside the container.

echo "██████╗  ██████╗ ██████╗     ███╗   ██╗███████╗██╗  ██╗██╗   ██╗███████╗"
echo "██╔══██╗██╔═══██╗██╔══██╗    ████╗  ██║██╔════╝╚██╗██╔╝██║   ██║██╔════╝"
echo "██████╔╝██║   ██║██████╔╝    ██╔██╗ ██║█████╗   ╚███╔╝ ██║   ██║███████╗"
echo "██╔══██╗██║   ██║██╔══██╗    ██║╚██╗██║██╔══╝   ██╔██╗ ██║   ██║╚════██║"
echo "██████╔╝╚██████╔╝██████╔╝    ██║ ╚████║███████╗██╔╝ ██╗╚██████╔╝███████║"
echo "╚═════╝  ╚═════╝ ╚═════╝     ╚═╝  ╚═══╝╚══════╝╚═╝  ╚═╝ ╚═════╝ ╚══════╝"
echo "                            INITIALIZING NEXUS"
echo ""

# 0. Check for Git
if ! command -v git &> /dev/null; then
    echo "[!] Git is not installed. Please install git first."
    exit 1
fi

# 1. Prepare Configuration
if [ ! -f master/config/.env ]; then
    echo "[*] Creating .env from template in master/config/..."
    if [ -f master/config/.env.template ]; then
        cp master/config/.env.template master/config/.env
    elif [ -f .env.template ]; then
        cp .env.template master/config/.env
    fi
    echo "[!] Please edit master/config/.env later to add your API keys!"
fi

if [ ! -f master/config/conf.yaml ]; then
    echo "[*] Creating master config from template..."
    cp master/config/conf.template.yaml master/config/conf.yaml
fi

# 2. Clone Core ROS 2 Packages
mkdir -p ros2_ws/src
cd ros2_ws/src

echo "[*] Cleaning up workspace and checking for core repositories..."
# Remove broken symlinks or external links to ensure we have local copies for the container
for repo in bob_llm bob_launch bob_topic_tools; do
    if [ -L "$repo" ]; then
        echo "    [!] Removing symlink for $repo to ensure local clone..."
        rm "$repo"
    fi
    
    if [ ! -d "$repo" ]; then
        echo "    [*] Cloning $repo..."
        git clone https://github.com/bob-ros2/$repo.git
    else
        echo "    [ok] $repo already exists."
    fi
done

cd ../..

# 3. Build Workspace
echo "[*] Building ROS 2 workspace (this might take a while)..."
# Clean up stale host-build artifacts if they exist in the root or workspace
echo "    [*] Removing stale build/install/log artifacts to prevent path conflicts..."
rm -rf build/ install/ log/
rm -rf ros2_ws/build/ ros2_ws/install/ ros2_ws/log/

# Switch to the workspace directory to build
cd ros2_ws
/bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"
cd ..

echo ""
echo "===================================================================="
echo "ONBOARDING COMPLETE."
echo "1. Run './mastermind.sh' to start the console."
echo "2. Select 'AWAKENING' to start the Mastermind node."
echo "3. Use './master/chat.sh --backend bob_llm' to chat with the Core."
echo "===================================================================="
