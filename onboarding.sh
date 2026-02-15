#!/bin/bash
set -e

# --- Experiment 7! Onboarding Script ---
# This script initializes the bob_nexus environment inside the container.

# Colors for the terminal
GREEN='\033[0;32m'
CYAN='\033[0;36m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BOLD='\033[1m'
NC='\033[0m'

echo -e "${CYAN}${BOLD}██████╗  ██████╗ ██████╗     ███╗   ██╗███████╗██╗  ██╗██╗   ██╗███████╗${NC}"
echo -e "${CYAN}${BOLD}██╔══██╗██╔═══██╗██╔══██╗    ████╗  ██║██╔════╝╚██╗██╔╝██║   ██║██╔════╝${NC}"
echo -e "${CYAN}${BOLD}██████╔╝██║   ██║██████╔╝    ██╔██╗ ██║█████╗   ╚███╔╝ ██║   ██║███████╗${NC}"
echo -e "${CYAN}${BOLD}██╔══██╗██║   ██║██╔══██╗    ██║╚██╗██║██╔══╝   ██╔██╗ ██║   ██║╚════██║${NC}"
echo -e "${CYAN}${BOLD}██████╔╝╚██████╔╝██████╔╝    ██║ ╚████║███████╗██╔╝ ██╗╚██████╔╝███████║${NC}"
echo -e "${CYAN}${BOLD}╚═════╝  ╚═════╝ ╚═════╝     ╚═╝  ╚═══╝╚══════╝╚═╝  ╚═╝ ╚═════╝ ╚══════╝${NC}"
echo -e "                            ${GREEN}INITIALIZING NEXUS${NC}"
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

# 2. Prepare Directories
mkdir -p entities
mkdir -p skills

# 3. Clone Core ROS 2 Packages
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

# HACK: Clear existing ROS workspace variables from the environment to avoid 
# annoying 'path doesn't exist' warnings from colcon build when we just deleted them.
# We then source only the base ROS 2 setup.
echo "    [*] Purging stale workspace variables for a clean build..."
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset PKG_CONFIG_PATH
unset PYTHONPATH

/bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --event-handlers console_cohesion+"
cd ..

echo ""
echo "===================================================================="
echo "ONBOARDING COMPLETE."
echo ""
echo -e "${BOLD}Next Steps:${NC}"
echo "1. Edit 'master/config/.env' to add your API keys."
echo "2. Run './mastermind.sh' to start the console."
echo "3. Inside the console, run 'Entity Setup' to create your Core."
echo "===================================================================="
echo ""

read -p "Would you like to start 'mastermind.sh' now? (y/n): " start_mm
if [[ $start_mm =~ ^[Yy]$ ]]; then
    exec ./mastermind.sh
else
    echo "Goodbye! Run ./mastermind.sh whenever you are ready."
fi
