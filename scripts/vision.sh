#!/usr/bin/env bash
#Pi_VISION

export BASEDIR=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
eval $($BASEDIR/hrtool -p|grep -E ^HR_WORKSPACE=)
export HR_WORKSPACE=$HR_WORKSPACE

. $HR_WORKSPACE/torch/install/bin/torch-activate

source $HR_WORKSPACE/HEAD/devel/setup.bash
export PYTHONPATH=$PYTHONPATH:$HR_WORKSPACE/openface/:$HR_WORKSPACE/dlib-18.18/dist/
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HR_WORKSPACE/CppMT/:$HR_WORKSPACE/emotime/build/src

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
    rosparam set use_sim_time true
    roslaunch robots_config face_tracker.launch pi_vision:=0 testing:=1
esac
# Enable visualzation tools
roslaunch robots_config perception.launch
