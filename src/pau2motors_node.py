#!/usr/bin/env python
#
# Robot PAU to motor mapping
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
#
import rospy
import Utils
from pau2motors.msg import pau
from Pau2Motors import Pau2Motors
import os
from optparse import OptionParser

def get_namespaces_containing(name_piece):
  return set([
    "/".join(param_path[:param_path.index(name_piece)])
    for param_path in [n.split("/") for n in rospy.get_param_names()]
    if name_piece in param_path
  ])

class Pau2MotorsNode:

  def _handle_pau_cmd(self, msg, pau2motors):
    pau2motors.consume(msg)

  def _subscribe_single(self, topicname, pau2motors_instance):
    rospy.Subscriber(
      topicname,
      pau,
      lambda msg: self._handle_pau_cmd(msg, pau2motors_instance)
    )

  def _build_msg_pipe(self, robot_yaml):

    # Build a dictionary that maps topic names to lists of motor yaml entries
    topicname2motor_sublist = {
      topicname: map(
        lambda name: robot_yaml["motors"][name],
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

  def __init__(self, config):
    rospy.init_node("pau2motors_node")
    self._build_msg_pipe(config)

parser = OptionParser()
if __name__ == '__main__':

  def print_help():
    msg = "Change namespace with `export ROS_NAMESPACE=<ns>`"
    ns = get_namespaces_containing(param_name)
    if len(ns) > 0:
      msg += "\nCurrent valid <ns>: %s." % ", ".join(ns)
    else:
      msg += "\nNo current valid <ns> found."
    msg += "\nCheck robots_config repo to get more config options."
    print(msg)

  #Config name to search in the parameter server.
  param_name = "pau2motors"
  config = rospy.get_param(param_name, None)

  print
  if config:
    Pau2MotorsNode(config)
    rospy.loginfo(
      "Loaded '%s' from param server.",
      rospy.get_namespace() + param_name
    )
    print_help()
    rospy.loginfo("Started")
    rospy.spin()
  else:
    print(
      "Couldn't find '%s' param in namespace '%s'." %
     (param_name, rospy.get_namespace())
    )
    print_help()
    print

