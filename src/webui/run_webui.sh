#!/usr/bin/env bash
#
# Run webui with chatbot only. This is mainly for test.
#

source ~/hansonrobotics/HEAD/devel/setup.bash
pidfile=$(mktemp)
echo "pid file $pidfile"

killme() {
  if [[ -f $pidfile ]]; then
    kill $(<"$pidfile") && rm $pidfile
  fi
  ps -ef|grep "python app/__init__.py -p 8000"|grep -v grep|awk '{print $2}'|xargs kill 2>/dev/null
}

trap 'killme' SIGINT EXIT

roslaunch robots_config rosbridge.launch &
echo $! >>$pidfile
until rostopic list ; do sleep 1; done
rosparam set /robot_name sophia
rosparam set lang en

export ROS_NAMESPACE=sophia
rosrun chatbot ai.py &
echo $! >>$pidfile
rosrun speech2command run.py &
echo $! >>$pidfile
rosrun webui fake_tts.py &
echo $! >>$pidfile
python app/__init__.py -p 8000 &
echo $! >>$pidfile

wait
