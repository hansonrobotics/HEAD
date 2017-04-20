#
# Factory for the pololu and dynamixel motors.
#
# The factory here will create classes that take numerical motor angles
# as input and publish the same as ROS messages for the specified topics.
#
# The expected format is ....
#
# ----------------------------------------------------------------
# Copyright (C) 2014, 2015 Hanson Robotics
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
# 02110-1301  USA

import Utils
import ShapekeyStore
from ros_pololu.msg import MotorCommand
from std_msgs.msg import Float64
import rospy

class HardwareBase:

  _topics = {}
  pub = None

  messageType = None

  def turn(self, angle):
    return NotImplementedError

  def getMessageType(self):
      return  self.messageType

  def __init__(self, args):
    ''' On construction parser classes are given the 'args' object (property
    'hardware' of 'binding') parsed from the robot config file in robots_config. '''

    if not self.__class__._topics.has_key(args["topic"]):
      self.__class__._topics[args["topic"]] = rospy.Publisher(
        self.getTopicName(args["topic"]), self.messageType, queue_size=30
      )
    self.pub = self.__class__._topics[args["topic"]]
    self.args = args

  def getTopicName(self, topic):
      ''' Default topic format '''
      return topic + "/command"

class Pololu(HardwareBase):
  messageType = MotorCommand

  def turn(self, angle):
    self.pub.publish(self.build_msg(angle))


  def build_msg(self, angle):
    msg = MotorCommand()
    msg.joint_name = self.args["name"]
    msg.position = angle
    # IF speed and acceleration is more than 1 default motor speed will be used
    msg.speed = 2
    msg.acceleration = 2
    return msg



class Dynamixel(HardwareBase):

  messageType = Float64

  def turn(self, angle):
    self.pub.publish(angle)

  def getTopicName(self, topic):
      ''' Default topic format '''
      return topic + "_controller/command"


_hardware_classes = {
  "pololu": Pololu,
  "dynamixel": Dynamixel
}

def build(yamlobj):
  return _hardware_classes[yamlobj["hardware"]](yamlobj)
