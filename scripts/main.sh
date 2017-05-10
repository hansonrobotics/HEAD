#!/usr/bin/env bash
#
# main.sh
#
# Hanson Robotics Robot Launch Entry
#

set -e

COLOR_INFO='\033[32m'
COLOR_WARN='\033[33m'
COLOR_ERROR='\033[31m'
COLOR_RESET='\033[0m'
info() {
    printf "${COLOR_INFO}[INFO] ${1}${COLOR_RESET}\n"
}
warn() {
    printf "${COLOR_WARN}[WARN] ${1}${COLOR_RESET}\n"
}
error() {
    printf "${COLOR_ERROR}[ERROR] ${1}${COLOR_RESET}\n"
}

show_help() {
cat << EOF
Usage: $0 OPTIONS <robot name>

Options are:
  --puppeteering|-p     Launch puppeteering
  --dev|-d              Run in dev mode
  --layout              Rearrange the applications layout
  --tracker tracker     Specify which tracker: pi_vision(default), cmt, or realsense
  --nogui               Don't show RVIZ and other unnecessary windows
  --body body           Specify body name (for dynamixel)
  --autoname            Automatically discover robot name by hardware
  --autobody            Automatically discover robot body by hardware
  --help|-h             Show this help

EOF
}

export CMT_TRACKING=0
export PI_VISION_TRACKING=0
export REALSENSE_TRACKING=0
export BASEDIR=$(dirname $(readlink -f ${BASH_SOURCE[0]}))

while [[ $# > 0 ]]; do
    case "$1" in
        --puppeteering|-p)
            export PUPPETEERING=1
            shift
            ;;
        --dev|-d)
            export DEV_MODE=1
            shift
            ;;
        --layout)
            export LAYOUT=1
            shift
            ;;
        --tracker)
          if [[ -z $2 ]]; then
            error "--tracker option is incorrect" >&2
            show_help
            exit 1
          fi
          case "$2" in
              cmt)
                  export CMT_TRACKING=1
                  ;;
              realsense)
                  export REALSENSE_TRACKING=1
                  export SALIENCY=1
                  ;;
              pi_vision)
                  export PI_VISION_TRACKING=1
                  ;;
              *)
                  error "--tracker option is incorrect" >&2
                  show_help
                  exit 1
                  ;;
          esac
          shift
          shift
          ;;
        --nogui)
            export NO_GUI=1
            shift
            ;;
        --body)
            export BODY=$2
            shift
            shift
            ;;
        --autoname)
            export AUTONAME=1
            shift
            ;;
        --autobody)
            export AUTOBODY=1
            shift
            ;;
        --help|-h)
            show_help
            exit 0
            ;;
        robot)
            export NAME="$1"
            shift
            ;;
        *)
            warn "Unknown argument $1"
            show_help
            exit 1
            ;;
    esac
done

if [[ -z $NAME ]]; then
    if [[ $AUTONAME == 1 ]]; then
        info "Discovering robot name"
        while [[ -z $NAME ]]; do
            NAME=$($BASEDIR/discover.py --dirname $BASEDIR/../src/robots_config|grep "robot name"|cut -d: -f2)
            if [[ -z $NAME ]]; then
                error "Can't discover robot name"
            fi
            sleep 1
        done
        warn "Automatically discover robot name $NAME"
    else
        error "Robot name is not set" >&2
        show_help
        exit 1
    fi
fi

if [[ -z $BODY ]]; then
    if [[ $AUTOBODY == 1 ]]; then
        info "Discovering robot body"
        while [[ -z $BODY ]]; do
            if [[ -e /dev/hr/body ]]; then
                for f in $(find /dev/hr/body -name dynamixel); do
                    BODY=$(echo $f | awk -F/ '{print $(NF-1)}')
                    break
                done
            fi
            if [[ -z $BODY ]]; then
                error "Can't discover Robot body name"
            fi
            sleep 1
        done
        warn "Automatically discover robot body name $BODY"
    else
        warn "Run without body"
    fi
fi

case $NAME in
    robot)
        info "Robot name is $NAME"
        export BLENDER_FILE=Sophia.blend
        ;;
    *)
        error "Robot name is incorrect" >&2
        exit 1
esac

if [[ ! "$(id -nG)" =~ .*"dialout".* ]]; then
    warn "You are not in the \"dialout\" group"
fi

source ~/.hr/env.sh

if [[ -z $HR_WORKSPACE ]]; then
    error "HR_WORKSPACE is not found"
    exit 1
else
    info HR_WORKSPACE=$HR_WORKSPACE
fi

if [[ -z $HR_PREFIX ]]; then
    error "HR_PREFIX is not found"
    exit 1
fi

export ROS_PYTHON_LOG_CONFIG_FILE="$BASEDIR/python_logging.conf"
export ROSCONSOLE_FORMAT='[${logger}][${severity}] [${time}]: ${message}'
export ROBOTS_CONFIG_DIR=$HR_WORKSPACE/HEAD/src/robots_config
export HR_CHARACTER_PATH=$HR_WORKSPACE/HEAD/src/chatbot/scripts/characters
export HR_CHATBOT_REVISION=$(git -C $HR_WORKSPACE/HEAD log -1 --format='%h' 2>/dev/null)
export LOCATION_SERVER_HOST=52.41.5.107
export LOCATION_SERVER_PORT=8004
export OPENWEATHERAPPID='1fe4806fc62327aa719a2bb215e983fe'

if [[ $(tmux ls 2>/dev/null) == ${NAME}* ]]; then
    tmux kill-session -t $NAME
    info "Killed session $NAME"
fi

ros_args="name:=$NAME robots_config_dir:=$ROBOTS_CONFIG_DIR"

if [[ $DEV_MODE == 1 ]]; then
    ros_args="$ros_args dev:=true"
fi

source /opt/hansonrobotics/ros/setup.bash
source $HR_WORKSPACE/HEAD/devel/setup.bash

if [[ $NO_GUI == 1 ]]; then
    ros_args="$ros_args gui:=false"
fi

if [[ $PUPPETEERING == 1 ]]; then
    ros_args="$ros_args puppeteering:=true"
fi

if [[ $PI_VISION_TRACKING == 0 && $REALSENSE_TRACKING == 0 && $CMT_TRACKING == 0 ]]; then
    export PI_VISION_TRACKING=1
fi

if [[ $PI_VISION_TRACKING == 1 ]]; then
    ros_args="$ros_args pi_vision:=true realsense:=false cmt:=false"
fi

if [[ $REALSENSE_TRACKING == 1 ]]; then
    ros_args="$ros_args pi_vision:=false realsense:=true cmt:=false"
fi

if [[ $CMT_TRACKING == 1 ]]; then
    ros_args="$ros_args pi_vision:=false realsense:=false cmt:=true"
fi

if [[ ! -z $BODY ]]; then
    dynamixel=/dev/hr/body/$BODY/dynamixel
    if [[ -e $dynamixel ]]; then
        ros_args="$ros_args dynamixel_usb:=true dynamixel_device:=${dynamixel}"
    else
        error "\"${dynamixel}\" doesn't exist"
        exit 1
    fi
fi

while rostopic list >/dev/null 2>&1; do error "roscore is running. Waiting." >&2; sleep 1; done

warn "ROS launch args \"${ros_args}\""

tmux new-session -d -s $NAME -n $NAME "roslaunch ${ROBOTS_CONFIG_DIR}/robot.launch $ros_args; $SHELL"
until rostopic list >/dev/null 2>&1; do sleep 1; info "Waiting for master"; done

info "Starting"

tmux new-window -n 'rosbridge_ssl' "roslaunch ${ROBOTS_CONFIG_DIR}/rosbridge_ssl.launch ssl:=true certfile:=$HR_WORKSPACE/HEAD/src/webui/backend/ssl/cert.crt keyfile:=$HR_WORKSPACE/HEAD/src/webui/backend/ssl/key.pem port:=9094"

WEBUI_SERVER=$HR_WORKSPACE/HEAD/src/webui/backend/entry.js
WEBPACK_OPTIONS="--optimize-minimize"
NODE_INTERPRETER="node"

if [[ $DEV_MODE == 1 ]]; then
  WEBPACK_OPTIONS="-w -dev"
  NODE_INTERPRETER="nodemon"
fi

tmux new-window -n 'webui' "cd $HR_WORKSPACE/HEAD/src/webui; webpack $WEBPACK_OPTIONS &
    $NODE_INTERPRETER $WEBUI_SERVER -p 4000 -c $ROBOTS_CONFIG_DIR -r $NAME -s &
    $NODE_INTERPRETER $WEBUI_SERVER -p 8000 -c $ROBOTS_CONFIG_DIR -r $NAME; $SHELL"

tmux new-window -n 'blender' "cd $HR_WORKSPACE/HEAD/src/blender_api && blender -y $BLENDER_FILE -P autostart.py; $SHELL"

if [[ -f $HR_PREFIX/bin/run_tts_server ]]; then
    tmux new-window -n 'tts' "$HR_PREFIX/bin/run_tts_server --voice_path=$HR_WORKSPACE/HEAD/src/tts/api; $SHELL"
else
    error "ttsserver is not installed. Run 'hr install head-python-ttsserver'"
fi

if [[ -f $HR_PREFIX/ros/lib/chatbot/run_server.py ]]; then
    tmux new-window -n 'chatbot' "$HR_PREFIX/ros/lib/chatbot/run_server.py; $SHELL"
else
    error "chatbot is not installed. Run 'hr install head-chatbot'"
fi

tmux new-window -n 'bash' "cd $BASEDIR; $SHELL"

if [[ $LAYOUT == 1 ]]; then
    $BASEDIR/layout.sh
fi

if [[ $DEV_MODE == 1 ]]; then
    rosrun dynamic_reconfigure dynparam set /${NAME}/tts_talker lipsync_blender true
fi

tmux attach;
