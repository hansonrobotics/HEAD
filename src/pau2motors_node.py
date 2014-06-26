#!/usr/bin/env python
import rospy
import Utils
from pau2motors.msg import *
from ros_pololu_servo.msg import servo_pololu
from Pau2Motors import Pau2Motors

class Pau2MotorsNode:

  def _handle_pau_cmd(self, msg, pau2motors):
    cmds = pau2motors.to_motor_cmds(msg)
    for cmd in cmds:
      self.pub_pololu.publish(cmd)

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

  def __init__(self, robot_yaml):

    rospy.init_node("pau2motors_node")
    self.pub_pololu = rospy.Publisher('cmd_pololu', servo_pololu, queue_size=10)

    self._build_msg_pipe(robot_yaml)

if __name__ == '__main__':
    Pau2MotorsNode(Utils.read_yaml("config/einstein.yaml"))
    rospy.loginfo("Started")
    rospy.spin()
