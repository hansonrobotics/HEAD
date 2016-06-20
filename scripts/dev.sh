#!/usr/bin/env bash
#
# dev.sh
#
# This script will launch ROS, WebUI, Blender and Behavior Tree.
#

set -e

export MAJOR_PROJECT=HEAD
export NAME=robot
BASEDIR=$(dirname $(readlink -f ${BASH_SOURCE[0]}))


export ROS_LOG_DIR="$HOME/.hr/log"
export ROS_PYTHON_LOG_CONFIG_FILE="$BASEDIR/python_logging.conf"
export ROSCONSOLE_FORMAT='[${logger}][${severity}] [${time}]: ${message}'

eval $($BASEDIR/hrtool -p|grep -E ^HR_WORKSPACE=)
echo HR_WORKSPACE=$HR_WORKSPACE

export HR_WORKSPACE=$HR_WORKSPACE

source $HR_WORKSPACE/$MAJOR_PROJECT/devel/setup.bash

export LAUNCH_DIR="$BASEDIR/../src/robots_config/launch/"
export OCBHAVE="$HR_WORKSPACE/opencog/ros-behavior-scripting/"

#CMT and OpenFACE eralated stuff
. $HR_WORKSPACE/torch/install/bin/torch-activate

source $HR_WORKSPACE/HEAD/devel/setup.bash
export PYTHONPATH=$PYTHONPATH:$HR_WORKSPACE/HEAD/src/vision/openface/:$HR_WORKSPACE/HEAD/src/vision/dlib-18.18/dist/
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HR_WORKSPACE/HEAD/src/vision/CppMT/:$HR_WORKSPACE/HEAD/src/vision/emotime/build/src

export tool="$1"
if [[ -z $tool ]]; then
    echo "Setting tool to pi_vision"
    tool=pi_vision
fi
case $tool in
    pi_vision)
    vision="true"
    ;;
    cmt)
    echo "Setting to use CMT"
    vision="false"
    ;;
esac
export vision
#End
#Kill existing session
if [[ $(tmux ls) == ${NAME}* ]]; then
    tmux kill-session -t $NAME
    echo "Killed session $NAME"
fi
cd $BASEDIR


tmux new-session -n 'roscore' -d -s $NAME 'roslaunch $LAUNCH_DIR/robot.launch basedir:=$LAUNCH_DIR name:=$NAME pi_vision:=$vision & rqt_console; $SHELL'

sleep 3
tmux new-window -n 'WebServers' 'python $HR_WORKSPACE/$MAJOR_PROJECT/src/webui/app/__init__.py -p 4000 -s \
    -c $HR_WORKSPACE/$MAJOR_PROJECT/src/webui/app/ssl/cert.crt  \
    -k $HR_WORKSPACE/$MAJOR_PROJECT/src/webui/app/ssl/key.pem &
python $HR_WORKSPACE/$MAJOR_PROJECT/src/webui/app/__init__.py -p 8000; $SHELL'

# blender
tmux new-window -n 'Blender' 'blender -y $HR_WORKSPACE/$MAJOR_PROJECT/src/blender_api/Sophia.blend -P \
    $HR_WORKSPACE/$MAJOR_PROJECT/src/blender_api/autostart.py; $SHELL'
tmux new-window -n 'Chat_server' 'cd $HR_WORKSPACE/$MAJOR_PROJECT/src/chatbot/src/server && python run.py; $SHELL'

# btree needs blender to be ready
sleep 8

# btree
export ROS_NAMESPACE=$NAME
if [[ -d $OCBHAVE ]]; then
    cd $OCBHAVE/src
    tmux new-window -n 'OCBTree' 'guile -l btree.scm; $SHELL'
    tmux new-window -n 'OCFaceTrack' 'python ../face_track/main.py; $SHELL'
fi
cd $BASEDIR
tmux new-window -n '$SHELL'
tmux attach;
