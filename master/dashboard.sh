#!/bin/bash

# --- bob_nexus Dashboard Wrapper ---

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"

# Source .env if exists
if [ -f "$ROOT_DIR/.env" ]; then
    set -a
    source "$ROOT_DIR/.env"
    set +a
fi

case "$1" in
    status)
        # Show once and exit
        python3 "$ROOT_DIR/master/core/dashboard_terminal.py" status
        ;;
    window)
        # Start native OS window and exit script
        echo "[*] Launching Nexus Native Dashboard..."
        python3 "$ROOT_DIR/master/core/dashboard_window.py" &
        exit 0
        ;;
    *)
        # Default: Loop in terminal
        python3 "$ROOT_DIR/master/core/dashboard_terminal.py" loop
        ;;
esac
