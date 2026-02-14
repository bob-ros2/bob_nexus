#!/bin/bash
# master/dev.sh - Maintenance Tool for Bob-Nexus

case "$1" in
    rebuild)
        echo "[*] Rebuilding Image..."
        docker build -t bob-nexus:latest .
        ;;
    start)
        echo "[*] Starting Nexus with named volumes..."
        docker run -it --name nexus \
          -v nexus_entities:/app/entities \
          -v nexus_memory:/app/memory \
          -v nexus_ws:/app/ros2_ws \
          -v nexus_config:/app/master/config \
          bob-nexus:latest
        ;;
    purge)
        echo "[!] WARNING: This will delete ONLY nexus_* volumes."
        read -p "Are you sure? (y/n) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            docker rm -f nexus 2>/dev/null
            docker volume rm nexus_entities nexus_memory nexus_ws nexus_config
            echo "[+] Nexus volumes purged."
        fi
        ;;
    *)
        echo "Usage: ./master/dev.sh {rebuild|start|purge}"
        ;;
esac

