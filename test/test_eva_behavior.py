#!/usr/bin/env python

import unittest
import os
import sys
import time
import ConfigParser

import rospy
import roslaunch
from roslaunch import core
import rostopic

from blender_api_msgs.msg import SetGesture, EmotionState, Target
from testing_tools import wait_for_message, wait_for_messages, ThreadWorker

PKG = 'eva_behavior'

def create_msg_listener(topic, topic_class, timeout):
    return ThreadWorker(target=wait_for_message,
                        args=(topic, topic_class, timeout))

class EvaBehaviorTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.behavior_config = ConfigParser.ConfigParser()
        config_file = os.path.join(os.path.dirname(__file__), '../behavior.cfg')
        assert os.path.isfile(config_file)
        cls.behavior_config.read(config_file)

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
        self.assertIsNotNone(msg)

        self.behavior_switch('btree_off')
        pub, msg_class = rostopic.create_publisher(
            '/camera/face_event', 'pi_face_tracker/FaceEvent', True)
        pub.publish(msg_class('new_face', 1))
        msg = wait_for_message(
            '/blender_api/set_gesture', SetGesture, timeout)
        self.assertIsNone(msg)

    def test_face_interaction1(self):
        timeout = 60
        self.behavior_switch('btree_on')
        positive_gestures = [
            x.strip() for x in self.behavior_config.get(
                    'gesture', 'positive_gestures').split(',')]

        pub, msg_class = rostopic.create_publisher(
            '/blender_api/available_emotion_states',
            'blender_api_msgs/AvailableEmotionStates', True)
        pub.publish(msg_class(['happy']))

        emo_msg_listener = create_msg_listener(
            '/blender_api/set_emotion_state', EmotionState, timeout)
        ges_msg_listener = create_msg_listener(
            '/blender_api/set_gesture', SetGesture, timeout)
        emo_msg_listener.start()
        ges_msg_listener.start()
        time.sleep(1) # wait for topic connection established

        pub, msg_class = rostopic.create_publisher(
            '/camera/face_event', 'pi_face_tracker/FaceEvent', True)
        pub.publish(msg_class('new_face', 1))

        emo_msg = emo_msg_listener.join()
        ges_msg = ges_msg_listener.join()
        self.assertIn(emo_msg.name, ['happy'])
        self.assertIn(ges_msg.name, positive_gestures)

    def test_face_interaction2(self):
        timeout = 60
        self.behavior_switch('btree_on')
        new_arrival_emotions = [
            x.strip() for x in self.behavior_config.get(
                    'emotion', 'new_arrival_emotions').split(',')]
        positive_gestures = [
            x.strip() for x in self.behavior_config.get(
                    'gesture', 'positive_gestures').split(',')]

        emo_msg_listener = create_msg_listener(
            '/blender_api/set_emotion_state', EmotionState, timeout)
        ges_msg_listener = create_msg_listener(
            '/blender_api/set_gesture', SetGesture, timeout)
        emo_msg_listener.start()
        ges_msg_listener.start()
        time.sleep(1) # wait for topic connection established

        pub, msg_class = rostopic.create_publisher(
            '/camera/face_event', 'pi_face_tracker/FaceEvent', True)
        pub.publish(msg_class('new_face', 1))

        emo_msg = emo_msg_listener.join()
        ges_msg = ges_msg_listener.join()
        self.assertIn(emo_msg.name, new_arrival_emotions)
        self.assertIn(ges_msg.name, positive_gestures)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'eva_behavior', EvaBehaviorTest)
    #unittest.main()

