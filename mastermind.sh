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

# 1. Source Environment (located in master/config/ for persistence)
ENV_FILE="master/config/.env"

if [ -f "$ENV_FILE" ]; then
    echo -e "[*] Loading environment from $ENV_FILE..."
    set -a
    source "$ENV_FILE"
    set +a
else
    echo -e "${RED}[!] WARNING: master/config/.env not found. Have you created it?${NC}"
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
    echo -e "${GREEN}[+] Requirements registry found.${NC}"
else
    echo -e "${YELLOW}[!] WARNING: requirements/ folder missing.${NC}"
fi

# Fixed Entity Count: Look into category subfolders
ENTITY_COUNT=$(find entities -mindepth 2 -maxdepth 2 -type d | wc -l)
echo -e "${BOLD}Current Swarm Status (${ENTITY_COUNT} Entities):${NC}"
./master/cli.sh status

if [ "$ENTITY_COUNT" -eq 0 ]; then
    echo -e "${YELLOW}${BOLD}[!] ATTENTION: No entities found in the Swarm.${NC}"
    echo -e "${YELLOW}Please run 'Entity Setup' (Option 6) to spawn your first Mastermind Core.${NC}"
fi
echo ""

show_menu() {
    # Check Mastermind Existence and Status
    local mm_exists=false
    if [ -d "entities/master/mastermind" ]; then
        mm_exists=true
    fi

    local mm_status="OFFLINE"
    local mm_manifest="entities/master/mastermind/manifest.json"
    if [ "$mm_exists" = true ] && [ -f "$mm_manifest" ] && grep -q '"status": "running"' "$mm_manifest"; then
        mm_status="RUNNING"
    fi

    echo -e "${CYAN}--- Control Console ---${NC}"
    if [ "$mm_exists" = false ]; then
        echo -e "1) ${YELLOW}[COGNITIVE GAP]${NC}: Core not spawned (Run Option 6 first)"
    elif [ "$mm_status" == "OFFLINE" ]; then
        echo -e "1) ${GREEN}AWAKENING${NC}: Launch Mastermind (Start Core)"
    else
        echo -e "1) ${RED}SILENCE${NC}: Stop Mastermind (Stop Core)"
    fi
    echo "2) Collective Status"
    echo "3) Manual Command Bridge (Advanced CLI)"
    echo "4) Integrated Dashboard (Live View)"
    echo "5) Nexus Chat (OAI Mode)"
    echo "6) Entity Setup (Guided Wizard)"
    echo "q) Exit to Void"
    echo ""
    echo -e "${CYAN}---------------------------------------------${NC}"
    read -p "Select priority [1-6, q]: " choice
}

while true; do
    show_menu
    case $choice in
        1)
            # Safety Check: Does the Mastermind entity even exist?
            if [ ! -d "entities/master/mastermind" ]; then
                 echo -e "${RED}[!] ERROR: The entity 'mastermind' does not exist yet.${NC}"
                 echo -e "${YELLOW}Please go to Option 6 and spawn a 'master' named 'mastermind'.${NC}"
                 sleep 2
                 continue
            fi

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
            echo -e "${YELLOW}Common Commands:${NC}"
            echo -e "  cli status         - Show all entities"
            echo -e "  cli up <name>      - Start an entity (e.g. cli up alice)"
            echo -e "  cli down <name>    - Stop an entity"
            echo -e "  cli refresh <name> - Regenerate system prompts"
            echo ""
            /bin/bash --rcfile master/nexus_shell.rc
            ;;
        4)
            echo -e "${CYAN}[*] Connecting to Hive Dashboard...${NC}"
            ./master/dashboard.sh
            ;;
        5)
            echo -e "${CYAN}[*] Opening Nexus Chat Interface...${NC}"
            ./master/chat.sh --backend oai
            ;;
        6)
            ./master/entity_setup.sh
            ;;
        q)
            echo -e "${YELLOW}Retreating to the shadows...${NC}"
            exit 0
            ;;
        *)
            echo -e "${RED}Invalid priority selection.${NC}"
            ;;
    esac
    echo -e "${CYAN}=============================================${NC}"
    echo ""
done
