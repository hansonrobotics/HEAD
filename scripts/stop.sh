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
    tmux kill-session -t $NAME
    echo "Killed session $NAME"
fi
