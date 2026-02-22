#!/bin/bash
# Swarm Orchestration Script (3.3)
# Usage: ./swarm.sh <spawn|up|down> [config_file]

COMMAND=$1
CONFIG_FILE=$2

# Internal default configuration
DEFAULT_CONFIG=$(cat <<EOF
mastermind master mastermind
manager master manager
neo coder coder
streamer infrastructure twitch_stream
face infrastructure face_marker
EOF
)

if [ -z "$COMMAND" ]; then
    echo "Usage: $0 <spawn|up|down> [config_file]"
    exit 1
fi

# Determine source (file or default)
if [ -n "$CONFIG_FILE" ] && [ -f "$CONFIG_FILE" ]; then
    SOURCE=$(cat "$CONFIG_FILE")
else
    SOURCE="$DEFAULT_CONFIG"
fi

echo "[*] Swarm Execution: $COMMAND"

# Process the list
while read -r name category template; do
    [ -z "$name" ] && continue
    case "$COMMAND" in
        spawn)
            echo "[+] Spawning $name ($category) using $template..."
            ./master/cli.sh spawn "$category" "$name" "$template"
            ;;
        up)
            echo "[+] Starting $name ($category)..."
            ./master/cli.sh up "$name" "$category"
            ;;
        down)
            echo "[+] Stopping $name ($category)..."
            ./master/cli.sh down "$name" "$category"
            ;;
        *)
            echo "Error: Unknown command '$COMMAND'"
            exit 1
            ;;
    esac
done <<< "$SOURCE"

echo "[*] Swarm operation '$COMMAND' completed."
