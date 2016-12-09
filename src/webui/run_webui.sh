#!/usr/bin/env bash
#
# Run webui with chatbot only. This is mainly for test.
#

set -e
source ~/hansonrobotics/HEAD/devel/setup.bash
pidfile=$(mktemp)
echo "pid file $pidfile"

killme() {
  if [[ -f $pidfile ]]; then
    kill $(<"$pidfile") || true
    rm $pidfile
  fi
}

trap 'killme' SIGINT EXIT

(nc -zv localhost 8001 1>/dev/null 2>&1) || (echo "ERROR: Chatbot server is not running" >&2 && exit 1)

roslaunch robots_config rosbridge.launch &
echo $! >>$pidfile
until rostopic list ; do sleep 1; done
rosparam set /robot_name sophia
rosparam set lang en

ROS_NAMESPACE=webui rosrun webui node_configuration.py &
echo $! >>$pidfile
export ROS_NAMESPACE=sophia
rosrun chatbot ai.py &
echo $! >>$pidfile
rosrun speech2command run.py &
echo $! >>$pidfile
rosrun webui fake_tts.py &
echo $! >>$pidfile
webpack -w &
echo $! >>$pidfile
cd backend/
nodemon entry.js -c /home/ubuntu/hansonrobotics/private_ws/src/robots_config -r sophia -p 8003 &
echo $! >>$pidfile

echo "Open http://localhost:8000/#interactions"
wait
