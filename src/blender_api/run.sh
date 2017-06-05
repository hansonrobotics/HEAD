#!/usr/bin/env bash

BLENDER_FILE=$1
if [[ -z $BLENDER_FILE ]]; then
    echo "Usage $0 <.blend file>"
    exit 1
fi
while true;
do
    echo "Start at $(date)"
    blender -y $BLENDER_FILE -P autostart.py
    echo "Blender is crashed. Restarting"
    sleep 1
done

