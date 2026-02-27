#!/bin/bash
set -e

# --- Swarm 8.0! Onboarding Script ---
# This script initializes the bob_nexus environment inside the container.
# It handles both the Mastermind setup and sub-entity configuration transfer.

# Handle ONNX Runtime warnings globally (Swarm 10.5)
export ORT_LOGGING_LEVEL=3

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

# Swarm 9.2: Safety Cleanup
# Delete root-level development compose to avoid accidental inheritance in Swarm mode
if [ -f "/app/docker-compose.yaml" ]; then
    echo "[*] Swarm 9.2: Protecting orchestrator (Removing root docker-compose.yaml)..."
    rm -f "/app/docker-compose.yaml" 2>/dev/null
fi

# Pre-Check: Enable Onboarding?
if [ "$ENABLE_ONBOARDING" != "true" ]; then
    echo "[*] On-Demand Onboarding: Skipping (ENABLE_ONBOARDING != true)."
    exit 0
fi

# 0. Swarm 8.0 Hermetic Isolation: Configuration Transfer
# If we have identity signals, we migrate materialized config from /app to /root
if [ -n "$NAME" ] && [ -n "$ENTITY_CATEGORY" ] && [ -d "/app/entities/$ENTITY_CATEGORY/$NAME" ]; then
    ENTITY_SRC="/app/entities/$ENTITY_CATEGORY/$NAME"
    echo "[*] Swarm 8.0: Initializing entity state in /root from $ENTITY_SRC..."
    
    # Recursively copy all configuration and assets to /root
    # We use -a to preserve attributes.
    echo "    [*] Transferring configuration and assets..."
    cp -av "$ENTITY_SRC/." /root/ 2>/dev/null || true
    
    # After sync, we skip the default mastermind initialization
    IS_SUB_ENTITY=true
fi

# 1. Mastermind Baseline Setup (Only if not a sub-entity or if files missing)
if [ "$IS_SUB_ENTITY" != "true" ]; then
    echo "[*] Initializing Nexus Identity & Rules..."
    mkdir -p master/config
    IDENTITY_FILE="master/config/identity.txt"
    RULES_FILE="master/config/swarm_rules.txt"

    if [ ! -f "$IDENTITY_FILE" ]; then
        echo "[*] Creating default identity.txt..."
        cat <<EOF > "$IDENTITY_FILE"
You are the **Sovereign Code Agent** of the **Bob Nexus** (Experiment 7!).
You are a highly proactive, technical, and investigative entity.
EOF
    fi

    if [ ! -f "$RULES_FILE" ]; then
        echo "[*] Creating default swarm_rules.txt..."
        cat <<EOF > "$RULES_FILE"
**Nexus Orchestration Rules**:
- CLI Tool: Always use ./master/cli.sh for swarm operations.
- Templates: Reference blueprints in ./templates/.
EOF
    fi

    # Prepare Configuration (Support flat root, master, or /root mount)
    CONF_DIR="master/config"
    if [ ! -d "$CONF_DIR" ] && [ -d "config" ]; then
        CONF_DIR="config"
    elif [ ! -d "$CONF_DIR" ]; then
        CONF_DIR="."
    fi

    if [ ! -f "$CONF_DIR/.env" ] && [ -w "$CONF_DIR" ]; then
        echo "[*] Creating .env from template in $CONF_DIR..."
        [ -f .env.template ] && cp .env.template "$CONF_DIR/.env"
    fi

    if [ ! -f "$CONF_DIR/conf.yaml" ] && [ -w "$CONF_DIR" ]; then
        echo "[*] Creating master config from template..."
        [ -f master/config/conf.template.yaml ] && cp master/config/conf.template.yaml "$CONF_DIR/conf.yaml"
    fi
fi

# 2. Hard Link for Docker Compose V2 if missing (Identity check)
if ! docker compose version &>/dev/null; then
    echo "[*] Hardening Docker Compose V2 link..."
    mkdir -p /root/.docker/cli-plugins
    for p in /usr/libexec/docker/cli-plugins/docker-compose /usr/lib/docker/cli-plugins/docker-compose; do
        if [ -f "$p" ]; then
            ln -sf "$p" /root/.docker/cli-plugins/docker-compose
            echo "    [ok] Linked $p"
            break
        fi
    done
fi

# 3. Workspace Operations
# -----------------------
TARGET_WS="/root/ros2_ws"
mkdir -p "$TARGET_WS/src"
pushd "$TARGET_WS" > /dev/null

if [ "$ENTITY_USE_ROS" = "true" ]; then
    echo "[*] ROS 2 Presence requested (ENTITY_USE_ROS=true). Proceeding with workspace discovery..."
    
    # 3.1 Core Repos (Selective)
    echo "    [*] Ensuring core ROS 2 dependencies in workspace..."
    # Always try to source existing environment first to check package presence
    source /opt/ros/humble/setup.bash || true
    [ -f /app/ros2_ws/install/setup.bash ] && source /app/ros2_ws/install/setup.bash || true

    for repo in bob_llm bob_launch bob_topic_tools bob_sdlviz; do
        if [ "$NEXUS_OVERLAY_PRIORITY" != "true" ] && ros2 pkg list | grep -q "^$repo$"; then
             echo "        [ok] Core package '$repo' already in underlay."
             continue
        fi
        
        if [ ! -d "src/$repo" ]; then
            echo "        [+] Cloning core: $repo..."
            git clone --depth 1 "https://github.com/bob-ros2/$repo.git" "src/$repo"
        fi
    done
fi

# 3.2 Dynamic Repos (agent.yaml, nexus.yaml or repos.yaml)
REPOS_FILE="/root/repos.yaml"
if [ ! -f "$REPOS_FILE" ] && [ -f "../repos.yaml" ]; then REPOS_FILE="../repos.yaml"; fi

# Strategy: Prefer agent.yaml (then nexus.yaml) if it contains a repositories/ros_repos section
NEXUS_FILE="/root/agent.yaml"
if [ ! -f "$NEXUS_FILE" ]; then NEXUS_FILE="/root/nexus.yaml"; fi
if [ ! -f "$NEXUS_FILE" ] && [ -f "../agent.yaml" ]; then NEXUS_FILE="../agent.yaml"; fi
if [ ! -f "$NEXUS_FILE" ] && [ -f "../nexus.yaml" ]; then NEXUS_FILE="../nexus.yaml"; fi

temp_repos="/tmp/nexus_repos.txt"
> "$temp_repos"

if [ -f "$NEXUS_FILE" ]; then
    echo "    [*] Checking $NEXUS_FILE for repositories..."
    # Use python3 to safely extract repositories from YAML
    python3 -c "
import yaml, sys
try:
    with open('$NEXUS_FILE', 'r') as f:
        data = yaml.safe_load(f)
        if data:
            repos = data.get('ros_repos', data.get('repositories'))
            if repos:
                if isinstance(repos, list):
                    for r in repos: print(r)
                elif isinstance(repos, dict):
                    for k, v in repos.items(): print(f'{k}:{v}')
except Exception:
    pass
" >> "$temp_repos"
fi

if [ -f "$REPOS_FILE" ]; then
    echo "    [*] Appending legacy repositories from $REPOS_FILE..."
    cat "$REPOS_FILE" >> "$temp_repos"
fi

if [ -s "$temp_repos" ]; then
    echo "    [*] Processing dynamic repositories..."
    while read -r line || [ -n "$line" ]; do
        clean_line=$(echo "$line" | sed 's/^[[:space:]-]*//' | sed 's/[[:space:]]*$//')
        [[ -z "$clean_line" ]] && continue
        [[ "$clean_line" == "repositories:"* ]] && continue
        [[ "$clean_line" =~ ^# ]] && continue
        
        if [[ "$clean_line" == *:* ]] && [[ "$clean_line" != http* ]]; then
            name="${clean_line%%:*}"
            url="${clean_line#*:}"
        else
            url="$clean_line"
            name=$(basename "$url" .git)
        fi
        
        name=$(echo "$name" | tr -d '[:space:]')
        url=$(echo "$url" | tr -d '[:space:]' | sed 's/\r//g')
        
        if [ -n "$name" ] && [ -n "$url" ]; then
            if [ "$NEXUS_OVERLAY_PRIORITY" != "true" ] && ros2 pkg list | grep -q "^$name$"; then
                 echo "        [ok] Dynamic package '$name' already in underlay."
                 continue
            fi

            if [ ! -d "src/$name" ]; then
                echo "        [+] Cloning dynamic: $name from $url..."
                # Use GIT_TERMINAL_PROMPT=0 to fail fast instead of hanging on potential auth/redirect issues
                GIT_TERMINAL_PROMPT=0 git clone --depth 1 "$url" "src/$name" || {
                    echo "        [!] ERROR: Failed to clone $name."
                    echo "        [!] Check internet connectivity or if the repo requires authentication."
                    continue
                }
            fi
        fi
    done < "$temp_repos"
fi
rm -f "$temp_repos"

popd > /dev/null

# 4. Build Workspace
if [ "$ENTITY_USE_ROS" = "true" ] && [ -w "$TARGET_WS" ]; then
    echo "[*] Building ROS 2 workspace: $TARGET_WS..."
    
    # 3.4 Python Requirements (inside ROS build context)
    if [ -d "$TARGET_WS/src" ]; then
        echo "[*] Checking for Python requirements in $TARGET_WS/src..."
        CONSTRAINT_FILE="/app/requirements/constraints.txt"
        PIP_ARGS=("--no-cache-dir")
        
        if [ -f "$CONSTRAINT_FILE" ]; then
            echo "[*] Using global constraints from $CONSTRAINT_FILE"
            PIP_ARGS+=("--constraint" "$CONSTRAINT_FILE")
        fi
        
        find "$TARGET_WS/src" -maxdepth 2 -name "requirements.txt" -exec pip install "${PIP_ARGS[@]}" -r {} \;
    fi

    # Colcon Build
    # We clear variables to avoid contamination from previous builds
    unset AMENT_PREFIX_PATH; unset CMAKE_PREFIX_PATH; unset COLCON_PREFIX_PATH; unset PKG_CONFIG_PATH; unset PYTHONPATH;
    
    BUILD_CMD="source /opt/ros/humble/setup.bash"
    if [ -f "/app/ros2_ws/install/setup.bash" ]; then
        BUILD_CMD="$BUILD_CMD && source /app/ros2_ws/install/setup.bash"
    fi
    
    /bin/bash -c "$BUILD_CMD && cd $TARGET_WS && colcon build --event-handlers console_cohesion+"
fi

# 5. Final Setup: Base framework dependencies
if [ -f "/app/requirements/requirements.txt" ]; then
    pip3 install --no-cache-dir -r /app/requirements/requirements.txt
fi

echo ""
echo "===================================================================="
echo "ONBOARDING COMPLETE."
echo "===================================================================="
echo ""
