#!/usr/bin/env bash
NAME=robot;

SESSION_INFO=$(tmux ls 2>/dev/null | grep $NAME)
if [[ ${SESSION_INFO} == ${NAME}* ]]; then
    tmux kill-session -t $NAME
fi

echo "Stopped"
