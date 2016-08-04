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
    export OC_AIML_FILE=~/.hr/cache/res/load-all.scm
    if [[ ! -f ${OC_AIML_FILE} ]] ; then
      echo "Downloading OpenCog AIML file"
      wget https://github.com/opencog/test-datasets/releases/download/current/aiml-current.tar.gz -O /tmp/aiml.tar.gz
      tar zxf /tmp/aiml.tar.gz -C ~/.hr/cache
      rm /tmp/aiml.tar.gz
    fi
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
tmux new-window -n 'webui' "python $HR_WORKSPACE/$MAJOR_PROJECT/src/webui/app/__init__.py -p 4000 -s \
    -c $HR_WORKSPACE/$MAJOR_PROJECT/src/webui/app/ssl/cert.crt  \
    -k $HR_WORKSPACE/$MAJOR_PROJECT/src/webui/app/ssl/key.pem &
python $HR_WORKSPACE/$MAJOR_PROJECT/src/webui/app/__init__.py -p 8000; $SHELL"

tmux new-window -n 'blender' "cd $HR_WORKSPACE/$MAJOR_PROJECT/src/blender_api && blender -y Sophia.blend -P autostart.py; $SHELL"
if [[ $OC_CHATBOT != 1 ]]; then
    tmux new-window -n 'chat_server' "cd $HR_WORKSPACE/$MAJOR_PROJECT/src/chatbot/scripts && python run_server.py; $SHELL"
fi
tmux new-window -n 'marytts' "~/.hr/tts/marytts/marytts-5.1.2/bin/marytts-server; $SHELL"

# btree needs blender to be ready
sleep 8

# OpenCog chatbot
if [[ $OC_CHATBOT == 1 ]]; then
  tmux new-window -n 'relex_server' "cd $HR_WORKSPACE/opencog/relex/ && bash opencog-server.sh; $SHELL"
  tmux new-window -n 'tel' "while true; do nc -zv localhost 17020 && break; sleep 1; done; expect $BASEDIR/load_scm.exp; $SHELL"
  tmux new-window -n 'oc-ctrl' "roslaunch opencog_control psi.launch; $SHELL"
  tmux new-window -n 'cog' "export ROS_NAMESPACE=$NAME; cd $OCBHAVE/src; guile -l btree-psi.scm; $SHELL"
else
  tmux new-window -n 'cog' "export ROS_NAMESPACE=$NAME; cd $OCBHAVE/src; guile -l btree.scm; $SHELL"
fi

tmux new-window -n 'fce' "export ROS_NAMESPACE=$NAME; cd $OCBHAVE/face_track; ./main.py; $SHELL"

sleep 4
tmux new-window -n 'cmt' "roslaunch robots_config face_tracker.launch ${pi_arg}; $SHELL"
tmux new-window -n 'bash' "cd $BASEDIR; $SHELL"
tmux attach;
