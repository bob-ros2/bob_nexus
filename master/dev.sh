#!/bin/bash
# master/dev.sh - Maintenance Tool for Bob-Nexus

# Get project root
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$( dirname "$SCRIPT_DIR" )"

case "$1" in
    rebuild)
        echo "[*] Rebuilding Image (bob-nexus:latest)..."
        cd "$PROJECT_ROOT" && docker build -t bob-nexus:latest .
        ;;
    start|enter)
        read -p "Enter container name [default: nexus]: " CNAME
        CNAME=${CNAME:-nexus}

        # Check if container exists
        if docker ps -a --format '{{.Names}}' | grep -Eq "^${CNAME}$"; then
            # If exists, check if running
            if docker ps --format '{{.Names}}' | grep -Eq "^${CNAME}$"; then
                echo "[*] Container '$CNAME' is already running. Entering..."
                docker exec -it "$CNAME" /bin/bash
            else
                echo "[*] Container '$CNAME' exists but is stopped. Waking it up..."
                docker start -ai "$CNAME"
            fi
        else
            echo "[*] Creating and starting NEW container '$CNAME'..."
            # Note: We use the name as prefix for volumes to keep them separate if desired?
            # Or keep shared volumes? The user said "ganz viele starten", which usually implies shared dev env or separate ones.
            # Let's use SHARED volumes for now as it's a "Nexus" view, but the container name is unique.
            docker run -it --name "$CNAME" \
              -v nexus_entities:/app/entities \
              -v nexus_memory:/app/memory \
              -v nexus_ws:/app/ros2_ws \
              -v nexus_config:/app/master/config \
              bob-nexus:latest
        fi
        ;;
    purge)
        read -p "Enter container/volume prefix to purge [default: nexus]: " PREFIX
        PREFIX=${PREFIX:-nexus}
        echo -e "\033[0;31m[!] WARNING: This will delete container '$PREFIX' and all '${PREFIX}_*' volumes.\033[0m"
        read -p "Are you sure? (y/n) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            echo "[*] Flushing '$PREFIX' from the system..."
            docker rm -f "$PREFIX" 2>/dev/null
            docker volume rm ${PREFIX}_entities ${PREFIX}_memory ${PREFIX}_ws ${PREFIX}_config 2>/dev/null
            echo "[+] Purge complete."
        fi
        ;;
    *)
        echo "Usage: ./master/dev.sh {rebuild|start|purge}"
        echo ""
        echo "  rebuild : Replays the docker build process."
        echo "  start   : Start a new or enter an existing container by name."
        echo "  purge   : Deletes container and its associated volumes."
        ;;
esac
