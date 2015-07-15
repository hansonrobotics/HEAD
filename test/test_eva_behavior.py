#!/usr/bin/env python

import unittest
import os
import sys
import time

import rospy
import roslaunch
from roslaunch import core

from std_msgs.msg import String
from blender_api_msgs.msg import SetGesture
from testing_tools import wait_for_message

PKG = 'eva_behavior'

class EvaBehaviorTest(unittest.TestCase):
    def setUp(self):
        self.run_id = 'test_eva_behavior'
        config = roslaunch.config.ROSLaunchConfig()
        self.node = core.Node(
            package='eva_behavior', node_type='main.py',
            name='eva_behavior')
        config.add_node(self.node)
        self.runner = roslaunch.launch.ROSLaunchRunner(
            self.run_id, config)
        self.runner.launch()
        rospy.init_node('test_eva_behavior')

    def tearDown(self):
        self.runner.stop()

    def test_btree_on_off(self):
        while not self.runner.is_node_running(self.node):
            time.sleep(0.2)

        if self.runner.is_node_running(self.node):
            pub = rospy.Publisher(
                '/behavior_switch', String, queue_size=30, latch=True)
            pub.publish(String('btree_on'))
            msg = wait_for_message('/blender_api/set_gesture', SetGesture, 30)
            self.assertIsNotNone(msg)
            pub.publish(String('btree_off'))
            msg = wait_for_message('/blender_api/set_gesture', SetGesture, 30)
            self.assertIsNone(msg)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'eva_behavior', EvaBehaviorTest)
    #unittest.main()

