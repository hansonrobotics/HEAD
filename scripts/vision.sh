#!/usr/bin/env bash
#Pi_VISION
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
    cmt)
    echo "Using CMT"
    roslaunch robots_config face_tracker.launch pi_vision:=0
    ;;
    cmt_test)
    echo "Using CMT in rosbag play mode"
    rosparam set use_sim_time true
    roslaunch robots_config face_tracker.launch pi_vision:=0 testing:=1
esac