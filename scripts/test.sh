#!/usr/bin/env bash
#
# test.sh
#
# Run all the tests
#

BASEDIR=$(dirname $(readlink -f ${BASH_SOURCE[0]}))

eval $($BASEDIR/hrtool -p|grep -E ^HR_WORKSPACE=)
echo HR_WORKSPACE=$HR_WORKSPACE
source /opt/ros/indigo/setup.bash
source $HR_WORKSPACE/HEAD/devel/setup.bash

export ROS_MASTER_URI=http://localhost:22422
export ROS_PYTHON_LOG_CONFIG_FILE="$BASEDIR/python_logging.conf"
export ROSCONSOLE_FORMAT='[${logger}][${severity}] [${time}]: ${message}'
export ROS_LOG_DIR="$HOME/.hr/log/tests"
export ROS_TEST_RESULTS_DIR="$ROS_LOG_DIR"

exit_status=true
export COVERAGE_RCFILE=$HR_WORKSPACE/HEAD/scripts/.coveragerc
cd $HR_WORKSPACE/HEAD
#rostest pi_face_tracker test_pi_face_tracker.test || exit_status=false
python src/hardware/pau2motors/test/test_pau2motors.py || exit_status=false
coverage run --rcfile $COVERAGE_RCFILE src/chatbot/test/test_chatbot.py || exit_status=false

coverage combine --rcfile $COVERAGE_RCFILE
coverage xml --rcfile $COVERAGE_RCFILE
coverage html --rcfile $COVERAGE_RCFILE

$exit_status
