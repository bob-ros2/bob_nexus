#!/bin/bash
# Mastermind CLI Wrapper
REAL_SCRIPTPATH=$(dirname "$(readlink -f "$0")")
python3 "$REAL_SCRIPTPATH/cli/cli.py" "$@"
