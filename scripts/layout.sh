#!/usr/bin/env bash

hash xdotool || sudo apt-get install xdotool

until (xdotool search --screen $DISPLAY --class blender); do echo "Waiting for blender"; sleep 1; done

WIDTH=100
HEIGHT=100
for WID in $(xdotool search --screen $DISPLAY --class blender); do
  xdotool windowsize $WID $WIDTH $HEIGHT
  xdotool windowmove $WID 0 0
done
