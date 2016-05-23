#!/usr/bin/env bash

export BASEDIR=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
eval $($BASEDIR/../hrtool -p|grep -E ^HR_WORKSPACE=)
echo HR_WORKSPACE=$HR_WORKSPACE

source $HR_WORKSPACE/HEAD/devel/setup.bash
export PYTHONPATH=$PYTHONPATH:$HR_WORKSPACE/openface/:$HR_WORKSPACE/dlib-18.18/dist/
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HR_WORKSPACE/CppMT/:$HR_WORKSPACE/emotime/build/src

roslaunch $BASEDIR/tracker-single-cam.launch
