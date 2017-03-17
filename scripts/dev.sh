#!/usr/bin/env bash
#
# dev.sh
#
# This script will launch ROS, WebUI, Blender and Behavior Tree.
#

set -e

show_help() {
cat << EOF
Usage: $0 [--cmt] [--oc] [--sa]
  --cmt
    Run CMT based face tracking
  --oc
    Run OpenCog based chatbot and behavior tree
  --sa
    Enable OpenCog NLP sentiment analysis, should run with --oc
  --st
    Enable saliency tracking.
EOF
}

while [[ $# > 0 ]]; do
    case "$1" in
        --cmt)
            export CMT_TRACKING=1
            shift
            ;;
        --oc)
            export OC_CHATBOT=1
            export OC_BTREE=1
            shift
            ;;
        --sa)
            export OC_SA=1
            shift
            ;;
        --st)
            export ENABLE_SALIENCY_TRACKING=1
            shift
            ;;
        --help|-h)
            show_help
            exit 0
            ;;
        *)
            echo "Unknown argument $1"
            show_help
            exit 1
            ;;
    esac
done

BASEDIR=$(dirname $(readlink -f ${BASH_SOURCE[0]}))

$BASEDIR/hrtool -r set_env
source $BASEDIR/env.sh
export MAJOR_PROJECT=HEAD
export NAME=robot
export ROS_PYTHON_LOG_CONFIG_FILE="$BASEDIR/python_logging.conf"
export ROSCONSOLE_FORMAT='[${logger}][${severity}] [${time}]: ${message}'
export LAUNCH_DIR="$BASEDIR/../src/robots_config/launch/"
export OC_LOG_LEVEL=info        # error, warn, info, debug and fine
export HR_CHARACTER_PATH=$HR_WORKSPACE/$MAJOR_PROJECT/src/chatbot/scripts/characters
export HR_CHATBOT_REVISION=$(git -C $HR_CHARACTER_PATH log -1 --format='%ci|%h' 2>/dev/null)
source $HR_WORKSPACE/$MAJOR_PROJECT/devel/setup.bash
echo HR_WORKSPACE=$HR_WORKSPACE

if [[ $(tmux ls 2>/dev/null) == ${NAME}* ]]; then
    tmux kill-session -t $NAME
    echo "Killed session $NAME"
fi

ros_args="basedir:=$LAUNCH_DIR name:=$NAME dev:=true"
pi_arg="pi_vision:=true"
if [[ $CMT_TRACKING == 1 ]]; then
    ros_args="$ros_args pi_vision:=false"
    pi_arg="pi_vision:=false"
fi

if [[ $OC_CHATBOT == 1 ]]; then
    ros_args="$ros_args oc_chatbot:=true"
    export OC_AIML_FILE=~/.hr/cache/oc_aiml/load.scm
    mkdir -p ~/.hr/cache/oc_aiml
    if [[ ! -f ${OC_AIML_FILE} ]] ; then
      echo "Downloading OpenCog AIML file"
      wget https://github.com/opencog/test-datasets/releases/download/current/oc_public_aiml.tar.gz -O /tmp/oc_public_aiml.tar.gz
      tar zxf /tmp/oc_public_aiml.tar.gz -C ~/.hr/cache/oc_aiml
      rm /tmp/oc_public_aiml.tar.gz
    fi
fi

if [[ $ENABLE_SALIENCY_TRACKING == 1 ]]; then
    ros_args="$ros_args saliency_tracking:=true"
fi

echo "ROS launch args \"${ros_args}\""

tmux new-session -n 'roscore'  -d -s $NAME "roslaunch ${LAUNCH_DIR}/robot.launch ${ros_args}; $SHELL"

#mandeep .. start cmt after btree
#sleep 3
#case $cmt in
#    true)
#    tmux new-window -n 'cmt' 'roslaunch robots_config face_tracker.launch pi_vision:=false; $SHELL'
#    ;;
#esac
sleep 3

ROBOT_CONFIG_DIR=$HR_WORKSPACE/$MAJOR_PROJECT/src/robots_config
WEBUI_SERVER=$HR_WORKSPACE/$MAJOR_PROJECT/src/webui/backend/entry.js
WEBPACK_OPTIONS="--optimize-minimize"
NODE_INTERPRETER="node"

if [[ $DEV_MODE == 1 ]]; then
  WEBPACK_OPTIONS="-w -dev"
  NODE_INTERPRETER="nodemon"
fi

tmux new-window -n 'webui' "cd $HR_WORKSPACE/$MAJOR_PROJECT/src/webui; webpack $WEBPACK_OPTIONS &
    $NODE_INTERPRETER $WEBUI_SERVER -p 4000 -c $ROBOT_CONFIG_DIR -r $NAME -s &
    $NODE_INTERPRETER $WEBUI_SERVER -p 8000 -c $ROBOT_CONFIG_DIR -r $NAME; $SHELL"

tmux new-window -n 'blender' "cd $HR_WORKSPACE/$MAJOR_PROJECT/src/blender_api && blender -y Sophia.blend -P autostart.py; $SHELL"
if [[ $OC_CHATBOT != 1 ]]; then
    tmux new-window -n 'chat_server' "cd $HR_WORKSPACE/$MAJOR_PROJECT/src/chatbot/scripts && python run_server.py; $SHELL"
fi
tmux new-window -n 'marytts' "~/.hr/tts/marytts/marytts-5.1.2/bin/marytts-server; $SHELL"

# btree needs blender to be ready
sleep 8

# OpenCog chatbot
if [[ $OC_CHATBOT == 1 ]]; then
  tmux new-window -n 'cog' "export ROS_NAMESPACE=$NAME; cd $OCBHAVE/src; guile -l btree-psi.scm; $SHELL"
  tmux new-window -n 'relex_server' "cd $HR_WORKSPACE/opencog/relex/ && bash opencog-server.sh; $SHELL"
  tmux new-window -n 'tel' "while true; do nc -zv localhost 17020 && break; sleep 1; done; expect $BASEDIR/load_scm.exp; $SHELL"
  tmux new-window -n 'oc-ctrl' "roslaunch opencog_control psi.launch; $SHELL"
else
  tmux new-window -n 'cog' "export ROS_NAMESPACE=$NAME; cd $OCBHAVE/src; guile -l btree.scm; $SHELL"
fi

tmux new-window -n 'orb' "export ROS_NAMESPACE=$NAME; cd $OCBHAVE/ros_sensors; ./main.py; $SHELL"

sleep 4
tmux new-window -n 'cmt' "roslaunch robots_config face_tracker.launch ${pi_arg}; $SHELL"
tmux new-window -n 'bash' "cd $BASEDIR; $SHELL"
tmux attach;
