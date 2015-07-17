#!/usr/bin/env python

import unittest
import os
import sys
import time

import rospy
import roslaunch
from roslaunch import core
import rostopic

from blender_api_msgs.msg import SetGesture, EmotionState
from testing_tools import wait_for_message

PKG = 'eva_behavior'

from rospy import SubscribeListener

class EvaBehaviorTest(unittest.TestCase):
    def setUp(self):
        self.run_id = 'test_eva_behavior'
        config = roslaunch.config.ROSLaunchConfig()
        self.node = core.Node(
            package='eva_behavior', node_type='main.py',
            name='Eva_behavior')
        config.add_node(self.node)
        self.runner = roslaunch.launch.ROSLaunchRunner(
            self.run_id, config)
        self.runner.launch()
        #rospy.init_node('test_eva_behavior')

    def tearDown(self):
        self.runner.stop()

    def behavior_switch(self, msg):
        while not self.runner.is_node_running(self.node):
            time.sleep(0.01)
        topic = '/behavior_switch'
        pub, msg_class = rostopic.create_publisher(
            topic, 'std_msgs/String', True)
        pub.publish(msg_class(msg))

    def test_btree_on_off(self):
        timeout = 60
        self.behavior_switch('btree_on')
        msg = wait_for_message(
            '/blender_api/set_gesture', SetGesture, timeout)
        rospy.loginfo("Got msg %s" % msg)
        self.assertIsNotNone(msg)

        self.behavior_switch('btree_off')
        msg = wait_for_message(
            '/blender_api/set_gesture', SetGesture, timeout)
        self.assertIsNone(msg)

    def test_face_interaction1(self):
        timeout = 30
        self.behavior_switch('btree_on')

        pub, msg_class = rostopic.create_publisher(
            '/blender_api/available_emotion_states',
            'blender_api_msgs/AvailableEmotionStates', True)
        pub.publish(msg_class(['happy']))

        pub, msg_class = rostopic.create_publisher(
            '/camera/face_event', 'pi_face_tracker/FaceEvent', True)
        pub.publish(msg_class('new_face', 1))

        ges_msg = wait_for_message(
            '/blender_api/set_gesture', SetGesture, timeout)
        self.assertIn(ges_msg.name, ['nod-1', 'nod-2'])
        emo_msg = wait_for_message(
            '/blender_api/set_emotion_state', EmotionState, timeout)
        self.assertIn(emo_msg.name, ['happy'])

    def test_face_interaction2(self):
        timeout = 30
        self.behavior_switch('btree_on')

        pub, msg_class = rostopic.create_publisher(
            '/camera/face_event', 'pi_face_tracker/FaceEvent', True)
        pub.publish(msg_class('new_face', 1))

        positive_emotions = ['happy', 'comprehending', 'engaged']
        emo_msg = wait_for_message(
            '/blender_api/set_emotion_state', EmotionState, timeout)
        self.assertIn(emo_msg.name, positive_emotions)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'eva_behavior', EvaBehaviorTest)
    #unittest.main()

