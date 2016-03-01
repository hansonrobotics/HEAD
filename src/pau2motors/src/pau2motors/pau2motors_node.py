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

import time
import logging
import rospy
from pau2motors.msg import pau
from Pau2Motors import Pau2Motors
from dynamic_reconfigure.server import Server
from pau2motors.cfg import Pau2motorsConfig

logger = logging.getLogger('hr.pau2motors.pau2motors_node')

class Pau2MotorsNode:

  def __init__(self):
    self.subs = []

  def _handle_pau_cmd(self, msg, pau2motors):
    pau2motors.consume(msg)

  def _subscribe_single(self, topicname, pau2motors_instance):
    sub = rospy.Subscriber(topicname, pau,
        lambda msg: self._handle_pau_cmd(msg, pau2motors_instance))
    logger.info("Subscribe {}".format(sub.name))
    return sub

  def _build_msg_pipe(self, config):
    if config is None:
      return

    for sub in self.subs:
      try:
        sub.unregister()
        logger.info("Unregister subscriber {}".format(sub.name))
        self.subs.remove(sub)
      except Exception as ex:
        logger.error(ex)
    for topicname, motor_names in config["topics"].items():
      motor_sublist = []
      for name in motor_names.split(';'):
        if name in config['motors']:
          motor_sublist.append(config['motors'][name])
        else:
          logger.error("motor {} has no configuration".format(name))
          continue
      sub = self._subscribe_single(topicname, Pau2Motors(motor_sublist))
      self.subs.append(sub)

  def reconfig(self, config, level):
    if config.reload:
      pau2motors_config = rospy.get_param("pau2motors", None)
      motors = rospy.get_param("motors", None)
      if pau2motors_config and motors:
        pau2motors_config["motors"] = dict([i["name"],i] for i in motors)
        self._build_msg_pipe(pau2motors_config)
      logger.info("Reload motor configurations")
      config.reload = False
    return config

if __name__ == '__main__':
  rospy.init_node("pau2motors")
  node = Pau2MotorsNode()
  Server(Pau2motorsConfig, node.reconfig)
  rospy.spin()
