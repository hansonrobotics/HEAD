#!/usr/bin/env python
import rospy
import Utils
from ros_faceshift.msg import *
from ros_pololu_servo.msg import servo_pololu
from Pau2Motors import Pau2Motors

class Pau2MotorsNode:

  def handle_face_pau(self, msg):
    cmds = self.pau2motors.to_motor_cmds(msg)
    for cmd in cmds:
      self.pub_pololu.publish(cmd)    

  def __init__(self, robot_yaml):
    self.pau2motors = Pau2Motors(robot_yaml)
    
    rospy.init_node("pau2motors_node")
    rospy.Subscriber("cmd_face_pau", fsMsgTrackingState, self.handle_face_pau)
    self.pub_pololu = rospy.Publisher('cmd_pololu', servo_pololu, queue_size=10)

if __name__ == '__main__':
    Pau2MotorsNode(Utils.read_yaml("einstein.yaml"))
    rospy.loginfo("Started")
    rospy.spin()