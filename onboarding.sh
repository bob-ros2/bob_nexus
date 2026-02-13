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

# 1. Prepare Configuration
if [ ! -f .env ]; then
    echo "[*] Creating .env from template..."
    cp .env.template .env
    echo "[!] Please edit .env later to add your API keys!"
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
# We need to source ROS first
/bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

echo ""
echo "===================================================================="
echo "ONBOARDING COMPLETE."
echo "1. Run './mastermind.sh' to start the console."
echo "2. Select 'AWAKENING' to start the Mastermind node."
echo "3. Use './master/cli.sh spawn assistant alice bob_llm' to create your first assistant."
echo "===================================================================="
