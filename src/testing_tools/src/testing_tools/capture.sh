#!/usr/bin/env bash

if [[ $# == 2 ]]; then
    FILENAME=$1
    DURATION=$2
else
    echo "Usage: $0 filename(.avi) duration(seconds)"
    exit 1
fi

if [[ -z $DISPLAY ]]; then
    export DISPLAY=:0
fi
WID=$(xdotool search --screen $DISPLAY --class blender)
WIDTH=924
HEIGHT=668
X=100
Y=100
xdotool windowsize $WID $WIDTH $HEIGHT
xdotool windowmove $WID $X $Y
xdotool windowactivate $WID

RECORDER=ffmpeg
FPS=25
GOP=15
RES=${WIDTH}x${HEIGHT}
XDISP=$DISPLAY+$X,$Y
VCODEC=libx264
CRF=18 # where 0 is lossless, 23 is default, and 51 is worst possible
PRESET=ultrafast
$RECORDER -y -f x11grab -an -t $DURATION -r $FPS -s $RES -i $XDISP \
    -vcodec $VCODEC -g $GOP -crf $CRF $FILENAME

xdotool windowminimize $WID
