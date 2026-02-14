#!bin/bash

if [ -n "$BASH_SOURCE" ]; then
  MMHOME=$(dirname "$(readlink -f "$BASH_SOURCE")")
else
  MMHOME=$(cd "$(dirname "$0")" && pwd)
fi

export PATH=$PATH:$MMHOME:$MMHOME/master:$MMHOME/skills/core/send_matrix/scripts
alias cli="$MMHOME/master/cli.sh"
alias chat="$MMHOME/master/chat.sh"
alias mm="$MMHOME/mastermind.sh"