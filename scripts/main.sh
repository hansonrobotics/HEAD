#!/usr/bin/env bash
#
# main.sh
#
# Hanson Robotics Robot Launch Entry
#

set -e

export BASEDIR=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
source $BASEDIR/common.sh

show_help() {
cat << EOF
Usage: $0 OPTIONS <robot name>

Options are:
  --puppeteering|-p <IP address>    Launch puppeteering. IP address is the one of the machine that runs Faceshift
  --dev|-d              Run in dev mode
  --layout              Rearrange the applications layout
  --tracker tracker     Specify which tracker: pi_vision(default), cmt, or realsense
  --nogui               Don't show RVIZ and other unnecessary windows
  --body body           Specify body name (for dynamixel)
  --autoname            Automatically discover robot name by hardware
  --autobody            Automatically discover robot body by hardware
  --dynamixel <device path>         Dynamixel device name
  --help|-h             Show this help

EOF
}

# cs 1024
# https 4000
# http 8000
# chatbot 8001
# ws 9090
# ttsserver 10001
check_ports 1024 4000 8000 8001 9090 10001
check_disk_usage
export CMT_TRACKING=0
export PI_VISION_TRACKING=0
export REALSENSE_TRACKING=0

while [[ $# > 0 ]]; do
    case "$1" in
        --puppeteering|-p)
            export PUPPETEERING=1
            ip=$2
            [[ -z $ip ]] && error "-p requires IP address" && exit 1
            if [[ $ip =~ ^[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}$ ]]; then
                export PUPPETEERING_IP=$ip
            else
                error "Invalid IP address \"$ip\""
                exit 1
            fi
            shift
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
        --dynamixel)
            if [[ -z $2 ]]; then
                error "--dynamixel option is incorrect" >&2
                show_help
                exit 1
            fi
            export DYNAMIXEL=1
            export DYNAMIXEL_DEVICE=$2
            shift
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

if [[ ! "$(id -nG)" =~ .*"dialout".* ]]; then
    warn "You are not in the \"dialout\" group"
fi

source ~/.hr/env.sh
source $HR_PREFIX/ros/setup.bash
export -f _ros_decode_path _ros_list_locations _ros_list_packages _ros_list_stacks _ros_location_find _ros_package_find _roscmd _roscomplete _roscomplete_exe _roscomplete_file _roscomplete_find _roscomplete_launch _roscomplete_node_transform _roscomplete_pkg _roscomplete_rosbag _roscomplete_rosclean _roscomplete_rosconsole _roscomplete_roscreate_pkg _roscomplete_rosmake _roscomplete_rosmsg _roscomplete_rosnode _roscomplete_rospack _roscomplete_rosparam _roscomplete_rosrun_transform _roscomplete_rosservice _roscomplete_rossrv _roscomplete_rostopic _roscomplete_roswtf _roscomplete_search_dir _roscomplete_sub_dir _roscomplete_test _rosfind _rossed rosawesome roscat roscd roscp rosd rosed rosls rospd rospython

if [[ $HR_ROLE == 'developer' ]]; then
    if [[ -z $HR_WORKSPACE ]]; then
        error "HR_WORKSPACE is not found"
        exit 1
    else
        info HR_WORKSPACE=$HR_WORKSPACE
    fi
fi

if [[ -z $HR_PREFIX ]]; then
    error "HR_PREFIX is not found"
    exit 1
fi

export ROBOTS_CONFIG_DIR=$BASEDIR/robots_config
export ROS_PYTHON_LOG_CONFIG_FILE="$BASEDIR/python_logging.conf"
export ROSCONSOLE_FORMAT='[${logger}][${severity}] [${time}]: ${message}'
export LOCATION_SERVER_HOST=52.41.5.107
export LOCATION_SERVER_PORT=8004
export OPENWEATHERAPPID='1fe4806fc62327aa719a2bb215e983fe'
export CITY_LIST_FILE=${HR_PREFIX}/data/city.list.json
export MARKY_MARKOV_DIR='/opt/hansonrobotics/tools/marky_markov'
export PKD_MODEL="$MARKY_MARKOV_DIR/models/pkd"
export DISPLAY=:0
export PY2ENV_DIR=/opt/hansonrobotics/py2env
export PY3ENV_DIR=/opt/hansonrobotics/py3env
export PY2PATH=$PY2ENV_DIR/lib/python2.7/site-packages:$PY2ENV_DIR/lib/python2.7/dist-packages:/usr/lib/python2.7/dist-packages
export PY3PATH=$PY3ENV_DIR/lib/python3.4/site-packages:$PY3ENV_DIR/lib/python3.4/dist-packages

if [[ -z $NAME ]]; then
    if [[ $AUTONAME == 1 ]]; then
        info "Discovering robot name"
        while [[ -z $NAME ]]; do
            NAME=$($BASEDIR/discover.py --dirname $ROBOTS_CONFIG_DIR|grep "robot name"|cut -d: -f2)
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

export HR_CHARACTER_PATH=$HR_WORKSPACE/HEAD/src/chatbot/scripts/characters
export HR_CHATBOT_REVISION=$(git -C $HR_WORKSPACE/HEAD log -1 --format='%h' 2>/dev/null)

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

if [[ $(tmux ls 2>/dev/null) == ${NAME}* ]]; then
    tmux kill-session -t $NAME
    info "Killed session $NAME"
fi

ros_args="name:=$NAME robots_config_dir:=$ROBOTS_CONFIG_DIR"

if [[ $DEV_MODE == 1 ]]; then
    ros_args="$ros_args dev:=true"
fi

if [[ $HR_ROLE == 'developer' ]]; then
    if [[ -f $HR_WORKSPACE/HEAD/devel/setup.bash ]]; then
        source $HR_WORKSPACE/HEAD/devel/setup.bash
    else
        error "Workspace is not built. Run 'hr build head'"
        exit 1
    fi
fi

if [[ $NO_GUI == 1 ]]; then
    ros_args="$ros_args gui:=false"
fi

if [[ $PUPPETEERING == 1 ]]; then
    ros_args="$ros_args puppeteering:=true puppeteering_ip:=$PUPPETEERING_IP"
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

if [[ $DYNAMIXEL == 1 ]]; then
   if [[ ! -e $DYNAMIXEL_DEVICE ]]; then
        error "Dynamixel device \"$DYNAMIXEL_DEVICE\" is not found"
        exit 1
   fi
   ros_args="$ros_args dynamixel_usb:=true dynamixel_device:=$DYNAMIXEL_DEVICE"
fi

if [[ ! -z $BODY ]]; then
    if [[ $DYNAMIXEL == 1 ]]; then
        error "Can't both specify --dynamixel with --body or --autobody"
        exit 1
    fi
    dynamixel=/dev/hr/body/$BODY/dynamixel
    if [[ -e $dynamixel ]]; then
        ros_args="$ros_args dynamixel_usb:=true dynamixel_device:=${dynamixel}"
    else
        error "Dynamixel device \"$dynamixel\" is not found"
        exit 1
    fi
fi

while rostopic list >/dev/null 2>&1; do error "roscore is running. Waiting." >&2; sleep 1; done

warn "ROS launch args:"
for arg in ${ros_args}; do warn $arg; done

tmux new-session -d -s $NAME -n $NAME "export PYTHONPATH=$PY2PATH:$PYTHONPATH && roslaunch ${ROBOTS_CONFIG_DIR}/robot.launch $ros_args; $SHELL"
until rostopic list >/dev/null 2>&1; do sleep 1; info "Waiting for master"; done

info "Starting"

if [[ $HR_ROLE == 'developer' ]]; then
    WEBUI_HOME=$HR_WORKSPACE/HEAD/src/webui
else
    WEBUI_HOME=/opt/hansonrobotics/ros/share/webui
fi

tmux new-window -n 'rosbridge_ssl' "roslaunch ${ROBOTS_CONFIG_DIR}/rosbridge_ssl.launch ssl:=true certfile:=$WEBUI_HOME/backend/ssl/cert.crt keyfile:=$WEBUI_HOME/backend/ssl/key.pem port:=9094"

WEBPACK_OPTIONS="--optimize-minimize"
NODE_INTERPRETER="node"

if [[ $DEV_MODE == 1 ]]; then
  WEBPACK_OPTIONS="-w -dev"
  NODE_INTERPRETER="nodemon"
fi

WEBUI_SERVER=$WEBUI_HOME/backend/entry.js
if [[ ! -f $WEBUI_SERVER ]]; then
    error "webui is not installed. Run 'hr install head-webui'"
fi

tmux new-window -n 'webui' "
    $NODE_INTERPRETER $WEBUI_SERVER -p 4000 -c $ROBOTS_CONFIG_DIR -r $NAME -s &
    $NODE_INTERPRETER $WEBUI_SERVER -p 8000 -c $ROBOTS_CONFIG_DIR -r $NAME; $SHELL"

tmux new-window -n 'blender' "export PYTHONPATH=$PYTHONPATH:$PY3PATH && cd $HR_PREFIX/tools/blender && blender -y $BLENDER_FILE -P autostart.py; $SHELL"

if [[ -f $HR_PREFIX/bin/run_tts_server ]]; then
    tmux new-window -n 'servers' "TTSSERVER=$HR_PREFIX/bin/run_tts_server supervisord -n -c $BASEDIR/supervisord.conf; $SHELL"
else
    error "ttsserver is not installed. Run 'hr install head-python-ttsserver'"
fi

if [[ -f $HR_PREFIX/ros/lib/chatbot/run_server.py ]]; then
    tmux new-window -n 'chatbot' "bash -c \"$HR_PREFIX/py2env/bin/python $HR_PREFIX/ros/lib/chatbot/run_server.py\"; $SHELL"
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

info "Processes are running background. To attach to this session, run 'tmux a'"
