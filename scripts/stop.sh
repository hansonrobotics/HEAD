#!/usr/bin/env bash
#
# dev.sh
#
# This script stops running software.
#
if [ -z "$1" ]; then
    if [ -z "$NAME" ]; then
        NAME=robot;
    fi
else
    NAME=$1
fi
if [[ $(tmux ls) == ${NAME}* ]]; then
    curl -G localhost:8001/v1.1/dump_history
    tmux kill-session -t $NAME
fi

echo "Stopped"
