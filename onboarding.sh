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
echo "[*] Workspace Repository Discovery..."

# Fallback to local if we are in an isolated context
TARGET_WS="ros2_ws"
if [ -d "/app/ros2_ws_local" ]; then
    TARGET_WS="/app/ros2_ws_local"
fi

mkdir -p "$TARGET_WS/src"
cd "$TARGET_WS"

# 3.1 Global Repos (only if building global ws)
if [ "$TARGET_WS" = "ros2_ws" ]; then
    echo "    [*] Checking core repositories..."
    for repo in bob_llm bob_launch bob_topic_tools bob_sdlviz; do
        if [ ! -d "src/$repo" ]; then
            echo "        [+] Cloning core: $repo..."
            git clone --depth 1 "https://github.com/bob-ros2/$repo.git" "src/$repo"
        fi
    done
fi

# 3.2 Dynamic Repos (repos.yaml)
REPOS_FILE="repos.yaml"
if [ -n "$ENTITY_CATEGORY" ] && [ -n "$NAME" ]; then
    # In container context, look where the entity root is
    POTENTIAL_REPOS="/app/entities/${ENTITY_CATEGORY}/${NAME}/repos.yaml"
    if [ -f "$POTENTIAL_REPOS" ]; then
        REPOS_FILE="$POTENTIAL_REPOS"
    fi
fi

if [ -f "$REPOS_FILE" ]; then
    echo "    [*] Processing dynamic repositories from $REPOS_FILE..."
    # Simple line-by-line parsing: "name: url"
    while IFS=": " read -r name url || [ -n "$name" ]; do
        # Skip comments and empty lines
        [[ "$name" =~ ^#.*$ ]] && continue
        [[ -z "$name" ]] && continue
        
        # Clean whitespace and trailing colons
        name=$(echo "$name" | tr -d '[:space:]' | tr -d ':')
        url=$(echo "$url" | tr -d '[:space:]' | sed 's/\r//g')
        
        if [ -n "$name" ] && [ -n "$url" ]; then
            if [ ! -d "src/$name" ]; then
                echo "        [+] Cloning dynamic: $name from $url..."
                git clone --depth 1 "$url" "src/$name"
            else
                echo "        [ok] $name already exists."
            fi
        fi
    done < "$REPOS_FILE"
fi

cd - > /dev/null

# 3. Build Workspace
echo "[*] Workspace Governance Check..."

# Determine target workspace
WS_DIR="ros2_ws"
IS_OVERLAY=false

if [ -d "/app/ros2_ws_local" ]; then
    echo "    [*] Isolated Context Detected: Preparing local overlay build..."
    WS_DIR="/app/ros2_ws_local"
    IS_OVERLAY=true
fi

# Check writability
if [ ! -w "$WS_DIR" ]; then
    if [ "$IS_OVERLAY" = true ]; then
        echo -e "${RED}[!] Local workspace $WS_DIR is NOT writable. Building aborted.${NC}"
        exit 1
    else
        echo -e "${YELLOW}[*] Global workspace $WS_DIR is read-only. Skipping master build (Consumer Mode).${NC}"
        exit 0
    fi
fi

echo "[*] Building ROS 2 workspace: $WS_DIR..."
rm -rf "$WS_DIR/build/" "$WS_DIR/install/" "$WS_DIR/log/"

# Build
echo "    [*] Purging stale workspace variables for a clean build..."
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset PKG_CONFIG_PATH
unset PYTHONPATH

BUILD_CMD="source /opt/ros/humble/setup.bash"
if [ "$IS_OVERLAY" = true ] && [ -f "/app/ros2_ws/install/setup.bash" ]; then
    echo "    [*] Using global workspace as underlay..."
    BUILD_CMD="$BUILD_CMD && source /app/ros2_ws/install/setup.bash"
fi
# 3.4 Python Requirements
if [ -d "$WS_DIR/src" ]; then
    echo "[*] Checking for Python requirements in $WS_DIR/src..."
    find "$WS_DIR/src" -maxdepth 2 -name "requirements.txt" -exec pip install --no-cache-dir -r {} \;
fi

/bin/bash -c "$BUILD_CMD && cd $WS_DIR && colcon build --event-handlers console_cohesion+"

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

# Non-interactive check (skip read if not in a TTY)
if [ -t 0 ]; then
    read -p "Would you like to start 'mastermind.sh' now? (y/n): " start_mm
    if [[ $start_mm =~ ^[Yy]$ ]]; then
        exec ./mastermind.sh
    else
        echo "Goodbye! Run ./mastermind.sh whenever you are ready."
    fi
else
    echo "[*] Non-interactive mode: Skipping manual terminal launch."
fi
