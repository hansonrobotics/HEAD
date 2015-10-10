#! /usr/bin/env python
#
# main.py - Main entry point for behaviors.
# Copyright (C) 2014  Hanson Robotics
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
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
#

import rospy
from general_behavior import Tree
import logging

if __name__ == "__main__":
    logger = logging.getLogger('hr.eva_behavior.main')
    rospy.init_node("Eva_Behavior")
    logger.info("Starting Behavior Node")
    tree = Tree()
