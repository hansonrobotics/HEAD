# This function assumes $ROSNODELIST holds the output of `rosnode list`
node_running () {
  echo $ROSNODELIST
  echo $ROSNODELIST | tr " " "\n" | grep -qxe .*/$1
  return $?
}

# Check ROS parameter against given regex
check_param_value() {
  echo `rosparam get $1` | grep -qxe $2
  return $?
}