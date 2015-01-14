# This function assumes $ROSNODELIST holds the output of `rosnode list`
node_running () {
  echo $ROSNODELIST
  echo $ROSNODELIST | tr " " "\n" | grep -qxe .*/$1
  return $?
}
