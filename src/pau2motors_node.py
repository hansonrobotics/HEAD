#!/usr/bin/env python
import rospy
import Utils
from pau2motors.msg import *
from Pau2Motors import Pau2Motors
import os

class Pau2MotorsNode:

  def _handle_pau_cmd(self, msg, pau2motors):
    pau2motors.consume(msg)

  def _subscribe_single(self, topicname, pau2motors_instance):
    rospy.Subscriber(
      topicname,
      fsMsgTrackingState,
      lambda msg: self._handle_pau_cmd(msg, pau2motors_instance)
    )

  def _build_msg_pipe(self, robot_yaml):

    # Build a dictionary that maps motor names to their yaml objects.
    motor_name2entry = {entry["name"]: entry for entry in robot_yaml["motors"]}

    # Build a dictionary that maps topic names to lists of motor yaml entries
    topicname2motor_sublist = {
      topicname: map(
        lambda name: motor_name2entry[name],
        motor_names.split(";")
      )
      for topicname, motor_names in robot_yaml["topics"].items()
    }

    # Convert lists of motor yaml entries to Pau2Motors instances.
    topicname2Pau2Motors = {
      topicname: Pau2Motors(motor_sublist)
      for topicname, motor_sublist in topicname2motor_sublist.items()
    }

    # Subscribe to topics
    for topicname, pau2motors_instance in topicname2Pau2Motors.items():
      self._subscribe_single(topicname, pau2motors_instance)

  def __init__(self):
    rospy.init_node("pau2motors_node")

    configname = rospy.get_param('~config', "einstein.yaml")
    configpath = os.path.join("config", configname) #Search in 'config' directory
    rospy.loginfo(
      "Loading %s\n(use _config:=robotname.yaml to load a different config)" % 
      configpath
    )

    self._build_msg_pipe(Utils.read_yaml(configpath))

if __name__ == '__main__':
    Pau2MotorsNode()
    rospy.loginfo("Started")
    rospy.spin()
