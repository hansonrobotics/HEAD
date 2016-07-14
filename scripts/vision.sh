#!/usr/bin/env bash
#Pi_VISION

export BASEDIR=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
eval $($BASEDIR/hrtool -p|grep -E ^HR_WORKSPACE=)
export HR_WORKSPACE=$HR_WORKSPACE
export HR_ENVFILE_PATH=$HR_ENVFILE_PATH
export HR_PREFIX=/opt/hansonrobotics
export HR_CACHE=~/.hr/cache
export VISION_TOOL_PREFIX=$HR_PREFIX/vision
export DLIB_DIR=$VISION_TOOL_PREFIX/dlib
export TORCH_DIR=$VISION_TOOL_PREFIX/torch
export OPENFACE_DIR=$VISION_TOOL_PREFIX/openface
export CPPMT_DIR=$VISION_TOOL_PREFIX/CppMT
export EMOTIME_DIR=$VISION_TOOL_PREFIX/emotime
export ROS_LOG_DIR="$HOME/.hr/log"
export OCBHAVE="$HR_WORKSPACE/opencog/ros-behavior-scripting"
export PYTHONPATH=$PYTHONPATH:$OCBHAVE/src:$OPENFACE_DIR:$DLIB_DIR/dlib-19.0/dist
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$CPPMT_DIR:$EMOTIME_DIR/build/src
source $TORCH_DIR/install/bin/torch-activate
source $HR_WORKSPACE/HEAD/devel/setup.bash
export DLIB_PATH=$DLIB_DIR/dlib-19.0

cd $HR_WORKSPACE/$PROJECT

include_dirs=($EMOTIME_DIR/src/{facedetector,utils,gaborbank,detector,training})
include_path=$(printf "%s:" "${include_dirs[@]}")
export CPLUS_INCLUDE_PATH=$CPLUS_INCLUDE_PATH:$CPPMT_DIR:$EMOTIME_DIR/include:$include_path

export tool="$1"
if [[ -z $tool ]]; then
    echo "Using Pi Vision"
    tool=pi_vision
fi
case $tool in
    pi_vision)
    echo "Using Pi"
    roslaunch robots_config face_tracker.launch pi_vision:=1
    ;;
    pi_viz)
    echo "Launching with RViZ"
    roslaunch robots_config face_tracker.launch pi_vision:=1 viz:=1
    ;;
    cmt)
    echo "Using CMT"
    roslaunch robots_config face_tracker.launch pi_vision:=0
    ;;
    cmt_test)
    echo "Using CMT in rosbag play mode"
    roslaunch robots_config face_tracker.launch pi_vision:=0 testing:=1
    ;;
    cmt_offline)
    echo "Train Data Sets and Modes"
    roslaunch robots_config face_tracker.launch pi_vision:=0 offline:=1
    ;;
esac
# Enable visualzation tools
#roslaunch robots_config perception.launch



