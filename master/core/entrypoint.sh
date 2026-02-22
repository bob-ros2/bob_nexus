#!/bin/bash
set -e

echo "[*] NEXUS Entity Boot: $NAME ($ENTITY_CATEGORY - $WORKSPACE_POLICY)"

# 0. Global Path-Relocation (Handle DooD mismatch)
if [ -n "$HOST_NEXUS_DIR" ] && [ "$HOST_NEXUS_DIR" != "/app" ]; then
    PARENT_DIR=$(dirname "$HOST_NEXUS_DIR")
    # Only attempt relocation if we can actually create the parent directory
    if [ ! -d "$HOST_NEXUS_DIR" ] && [ -w "/" ] || [ -d "$PARENT_DIR" ] && [ -w "$PARENT_DIR" ]; then
        echo "[*] Path-Relocation: Linking $HOST_NEXUS_DIR -> /app..."
        mkdir -p "$PARENT_DIR" 2>/dev/null || true
        ln -s /app "$HOST_NEXUS_DIR" 2>/dev/null || true
    else
        echo "[!] Path-Relocation: Skipping (Permission denied for parent of $HOST_NEXUS_DIR)"
    fi
fi

# 0.1 Load Local Environment
if [ -f /root/.env ]; then
    echo "[*] Sourcing local environment from /root/.env..."
    set -a
    source /root/.env
    set +a
fi

# 1. Check for Onboarding (handles both initial setup and on-demand building)
if [ -f /app/onboarding.sh ]; then
    /app/onboarding.sh
fi

# 1.1 Fast Python Dependency Check
if [ -d "/root/ros2_ws/src" ]; then
    echo "[*] Ensuring Python dependencies are met in /root..."
    find "/root/ros2_ws/src" -maxdepth 2 -name "requirements.txt" -exec pip install --no-cache-dir -r {} \;
fi

# 2. Source ROS Environments
if [ "$ENTITY_USE_ROS" = "true" ]; then
    echo "[*] Sourcing ROS 2 Environments..."
    # 1. Base ROS 2 Humble Environment
    source /opt/ros/humble/setup.bash

    # 2. Framework Underlay (Shadow mount from host or internal /app)
    if [ -f /opt/ros/underlay/local_setup.bash ]; then
        echo "[*] Sourcing Host-Shadowing Underlay (/opt/ros/underlay)..."
        source /opt/ros/underlay/local_setup.bash
    elif [ -f /app/ros2_ws/install/local_setup.bash ]; then
        echo "[*] Sourcing Global Framework Workspace (/app)..."
        source /app/ros2_ws/install/local_setup.bash
    fi

    # 3. Local Workspace Overlay (/root)
    if [ -f /root/ros2_ws/install/local_setup.bash ]; then
        echo "[*] Sourcing Local Workspace Overlay (/root)..."
        source /root/ros2_ws/install/local_setup.bash
    fi
else
    echo "[*] Non-ROS Runtime Mode (ENTITY_USE_ROS != true)."
fi

# 3. Launch
if [ -n "$ENTITY_ENTRYPOINT" ] && [ "$ENTITY_ENTRYPOINT" != "/app/master/core/entrypoint.sh" ]; then
    echo "[*] Executing Custom Entrypoint: $ENTITY_ENTRYPOINT"
    exec /bin/bash -c "$ENTITY_ENTRYPOINT"
elif [ "$ENTITY_USE_ROS" = "true" ]; then
    echo "[*] Launching ROS 2 Generic System..."
    # Priority: bob_launch.yaml then agent.yaml in entity root
    launch_config=""
    if [ -f "/root/bob_launch.yaml" ]; then
        launch_config="config:=/root/bob_launch.yaml"
    elif [ -f "/root/agent.yaml" ]; then
        launch_config="config:=/root/agent.yaml"
    fi
    exec ros2 launch bob_launch generic.launch.py $launch_config
else
    echo "[!] No ENTRYPOINT defined and Non-ROS mode. Falling back to persistent idle mode."
    echo "[*] Keeping container alive via tail -f /dev/null..."
    exec tail -f /dev/null
fi
