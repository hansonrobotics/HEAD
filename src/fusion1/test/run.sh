#!/usr/bin/env bash

set -e

SCRIPTDIR=~/hansonrobotics/HEAD/scripts
BASEDIR=~/hansonrobotics/HEAD/src/fusion1/test

. ~/hansonrobotics/HEAD/devel/setup.bash

$SCRIPTDIR/hrtool -r set_env
source $SCRIPTDIR/env.sh
export MAJOR_PROJECT=HEAD
export NAME=robot
export ROSCONSOLE_FORMAT='[${logger}][${severity}] [${time}]: ${message}'
export LAUNCH_DIR=$BASEDIR
export OC_LOG_LEVEL=info        # error, warn, info, debug and fine
source $HR_WORKSPACE/$MAJOR_PROJECT/devel/setup.bash
echo HR_WORKSPACE=$HR_WORKSPACE

if [[ $(tmux ls 2>/dev/null) == ${NAME}* ]]; then
    tmux kill-session -t $NAME
    echo "Killed session $NAME"
fi

ros_args="basedir:=$BASEDIR name:=$NAME"

echo "ROS launch args \"${ros_args}\""

tmux new-session -n 'roscore' -d -s $NAME "roslaunch ${LAUNCH_DIR}/robot.launch ${ros_args}; $SHELL"
until rostopic list >/dev/null 2>&1; do sleep 1; done

tmux new-window -n 'fusion' "export ROS_NAMESPACE=/${NAME}; rosrun fusion1 fusion.py; $SHELL"

tmux new-window -n 'bash' "cd $BASEDIR; $SHELL"

tmux attach;
