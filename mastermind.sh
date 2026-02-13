#!/bin/bash

# --- Experiment 7! Orchestrator Console ---

# Colors for the terminal
GREEN='\033[0;32m'
CYAN='\033[0;36m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BOLD='\033[1m'
NC='\033[0m' # No Color

export BOB_NEXUS_DIR=$(pwd)
clear

echo -e "${CYAN}${BOLD}"
echo "  ██████╗  ██████╗ ██████╗     ███╗   ██╗███████╗██╗  ██╗██╗   ██╗███████╗"
echo "  ██╔══██╗██╔═══██╗██╔══██╗    ████╗  ██║██╔════╝╚██╗██╔╝██║   ██║██╔════╝"
echo "  ██████╔╝██║   ██║██████╔╝    ██╔██╗ ██║█████╗   ╚███╔╝ ██║   ██║███████╗"
echo "  ██╔══██╗██║   ██║██╔══██╗    ██║╚██╗██║██╔══╝   ██╔██╗ ██║   ██║╚════██║"
echo "  ██████╔╝╚██████╔╝██████╔╝    ██║ ╚████║███████╗██╔╝ ██╗╚██████╔╝███████║"
echo "  ╚═════╝  ╚═════╝ ╚═════╝     ╚═╝  ╚═══╝╚══════╝╚═╝  ╚═╝ ╚═════╝ ╚══════╝"
echo -e "                            ${GREEN}EXPERIMENT 7! - THE AWAKENING${NC}"
echo ""

# 1. Source Environment
if [ -f .env ]; then
    echo -e "[*] Loading environment from .env..."
    # Source .env but keep it simple
    set -a
    source .env
    set +a
else
    echo -e "${RED}[!] WARNING: .env file not found. Have you created it?${NC}"
fi

# 2. Source ROS 2 Environment
ROS_SOURCE=$(python3 master/core/env_helper.py "$(pwd)")
if [ ! -z "$ROS_SOURCE" ]; then
    echo -e "[*] Sourcing ROS 2 workspaces..."
    eval "$ROS_SOURCE"
fi

if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}[!] CRITICAL: ROS 2 not detected even after sourcing setups in conf.yaml.${NC}"
    echo -e "${YELLOW}Please check master/config/conf.yaml and ensure paths are correct.${NC}"
else
    echo -e "${GREEN}[+] ROS 2 environment active.${NC}"
fi

# 3. Check Dependencies (Quick Check)
echo -e "[*] Reviewing neural paths (dependencies)..."
if [ -d requirements ]; then
    # This is a bit complex for a shell script, so we just acknowledge it
    echo -e "${GREEN}[+] Requirements registry found.${NC}"
else
    echo -e "${YELLOW}[!] WARNING: requirements/ folder missing.${NC}"
fi

echo ""
echo -e "${BOLD}Current Entities Status:${NC}"
./master/cli.sh status
echo ""

show_menu() {
    # Check Mastermind Status
    local mm_status="OFFLINE"
    local mm_manifest="entities/master/mastermind/manifest.json"
    if [ -f "$mm_manifest" ] && grep -q '"status": "running"' "$mm_manifest"; then
        mm_status="RUNNING"
    fi

    echo -e "${CYAN}--- Control Console ---${NC}"
    if [ "$mm_status" == "OFFLINE" ]; then
        echo -e "1) ${GREEN}AWAKENING${NC}: Launch Mastermind (Start Core)"
    else
        echo -e "1) ${RED}SILENCE${NC}: Stop Mastermind (Stop Core)"
    fi
    echo "2) Collective Status"
    echo "3) Enter Hive Logic (Manual CLI)"
    echo "4) Integrated Dashboard (Live View)"
    echo "q) Exit to Void"
    echo ""
    read -p "Select priority [1-4, q]: " choice
}

while true; do
    show_menu
    case $choice in
        1)
            mm_manifest="entities/master/mastermind/manifest.json"
            if [ -f "$mm_manifest" ] && grep -q '"status": "running"' "$mm_manifest"; then
                echo -e "${RED}[*] Silencing Mastermind...${NC}"
                ./master/cli.sh down mastermind
            else
                echo -e "${GREEN}[*] Initiating consciousness for the Core...${NC}"
                ./master/cli.sh up mastermind
            fi
            ;;
        2)
            ./master/cli.sh status
            ;;
        3)
            echo -e "${CYAN}Entering Manual CLI Mode. Type 'exit' to return.${NC}"
            /bin/bash
            ;;
        4)
            echo -e "${CYAN}[*] Connecting to Hive Dashboard...${NC}"
            ./master/dashboard.sh
            ;;
        q)
            echo -e "${YELLOW}Retreating to the shadows...${NC}"
            exit 0
            ;;
        *)
            echo -e "${RED}Invalid priority selection.${NC}"
            ;;
    esac
    echo ""
done
