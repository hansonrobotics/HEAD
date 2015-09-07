#!/usr/bin/env python

import unittest
import rospy
import os
import sys
import time

import roslaunch
import rostopic
import rospkg
from roslaunch import core

from std_msgs.msg import String
from testing_tools import wait_for, MessageQueue
from roslaunch import nodeprocess
nodeprocess._TIMEOUT_SIGINT = 2
nodeprocess._TIMEOUT_SIGTERM = 1

CWD = os.path.abspath(os.path.dirname(__file__))
PKG = 'chatbot'

class ChatbotTest(unittest.TestCase):

    def setUp(self):
        self.run_id = 'test_chatbot'
        config = roslaunch.config.ROSLaunchConfig()
        config.add_node(
            core.Node(
                package='chatbot', node_type='ai.py', args='%s/../aiml' % CWD,
                name='chatbot_ai')
            )
        robots_config_path = rospkg.RosPack().get_path('robots_config')
        config.add_param(core.Param(
            '/chatbot_ai/properties', '%s/sophia/bot.properties' % robots_config_path))
        self.runner = roslaunch.launch.ROSLaunchRunner(
            self.run_id, config, is_rostest=True)
        self.runner.launch()

        for node in config.nodes:
            wait_for('%s/%s' % (node.namespace, node.name))

        self.msg = None
        time.sleep(10) # wait for chatbot init done

    def tearDown(self):
        self.runner.stop()

    def test_prologue(self):
        timeout = 5
        queue = MessageQueue()
        queue.subscribe('/chatbot_responses', String)
        pub, msg_class = rostopic.create_publisher(
            '/chatbot_speech', 'chatbot/ChatMessage', True)
        pub.publish(msg_class('hi', 100))
        msg = queue.get(timeout=timeout)
        self.assertIn(msg.data, ['Hi there!', 'How are you?'])
        pub.publish(msg_class('what\'s your name', 100))
        msg = queue.get(timeout=timeout)
        self.assertIn('Sophia', msg.data)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'chatbot', ChatbotTest)

