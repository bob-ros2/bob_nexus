#!/bin/bash
# --- Experiment 7! Entity Setup Wizard ---

# Colors for the terminal
GREEN='\033[0;32m'
CYAN='\033[0;36m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BOLD='\033[1m'
NC='\033[0m'

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$( dirname "$SCRIPT_DIR" )"

echo -e "${CYAN}${BOLD}Nexus Entity Setup Wizard${NC}"
echo "---------------------------"

show_setup_menu() {
    echo -e "1) ${GREEN}Quick Start${NC}: Create 'mastermind' entity (The Core)"
    echo -e "2) ${CYAN}Custom Assistant${NC}: Create a new assistant (e.g., alice, bob)"
    echo -e "3) ${YELLOW}Inference Provider${NC}: Create a llama.cpp GGUF server (CUDA)"
    echo "q) Back to Console"
    echo ""
    read -p "Select setup path [1-2, q]: " setup_choice
}

create_master() {
    echo -e "[*] Spawning the Mastermind Core..."
    "$SCRIPT_DIR/cli.sh" spawn master mastermind bob_llm
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}[+] Mastermind Core created successfully!${NC}"
        echo -e "[*] You can now start it from the main console menu."
    else
        echo -e "${RED}[!] Failed to create Mastermind Core.${NC}"
    fi
}

create_assistant() {
    read -p "Enter assistant name (e.g. alice): " ASST_NAME
    if [ -z "$ASST_NAME" ]; then
        echo -e "${RED}[!] Name cannot be empty.${NC}"
        return
    fi
    
    echo -e "[*] Spawning assistant '$ASST_NAME'..."
    "$SCRIPT_DIR/cli.sh" spawn assistant "$ASST_NAME" bob_llm
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}[+] Assistant '$ASST_NAME' created!${NC}"
    else
        echo -e "${RED}[!] Failed to create assistant.${NC}"
    fi
}
create_inference() {
    read -p "Enter inference entity name [default: brain]: " INF_NAME
    INF_NAME=${INF_NAME:-brain}
    
    echo -e "[*] Spawning inference provider '$INF_NAME'..."
    "$SCRIPT_DIR/cli.sh" spawn infrastructure "$INF_NAME" inference/llama_cpp
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}[+] Inference Provider '$INF_NAME' created!${NC}"
        echo -e "[*] Run './mastermind.sh' -> 'Collective Status' to see it."
    else
        echo -e "${RED}[!] Failed to create inference provider.${NC}"
    fi
}

while true; do
    show_setup_menu
    case $setup_choice in
        1)
            create_master
            ;;
        2)
            create_assistant
            ;;
        3)
            create_inference
            ;;
        q)
            exit 0
            ;;
        *)
            echo -e "${RED}Invalid selection.${NC}"
            ;;
    esac
    echo ""
    read -n 1 -s -r -p "Press any key to continue..."
    echo ""
done
