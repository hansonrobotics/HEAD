#!/usr/bin/env python

import unittest
import os
import time
import ConfigParser

import roslaunch
from roslaunch import core
import rostopic

from blender_api_msgs.msg import SetGesture, EmotionState
from testing_tools.misc import MessageQueue
from roslaunch import nodeprocess
nodeprocess._TIMEOUT_SIGINT = 2
nodeprocess._TIMEOUT_SIGTERM = 1

PKG = 'eva_behavior'

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

    def tearDown(self):
        self.runner.stop()

    def behavior_switch(self, msg):
        while not self.runner.is_node_running(self.node):
            time.sleep(0.01)
        topic = '/behavior_switch'
        pub, msg_class = rostopic.create_publisher(
            topic, 'std_msgs/String', True)
        pub.publish(msg_class(msg))

    def test_btree_off(self):
        queue = MessageQueue()
        queue.subscribe('/blender_api/set_gesture', SetGesture)
        time.sleep(1)
        self.assertTrue(queue.queue.empty())
        self.behavior_switch('btree_on')
        msg = queue.get()
        self.assertTrue(msg is not None)

    def test_btree_on(self):
        timeout = 30
        queue = MessageQueue()
        queue.subscribe('/blender_api/set_gesture', SetGesture)
        time.sleep(1)
        self.behavior_switch('btree_off')
        pub, msg_class = rostopic.create_publisher(
            '/camera/face_event', 'pi_face_tracker/FaceEvent', True)
        pub.publish(msg_class('new_face', 1))
        time.sleep(timeout)
        self.assertTrue(queue.queue.empty())

    @unittest.skip("Not stable")
    def test_face_interaction1(self):
        positive_gestures = [
            x.strip() for x in self.behavior_config.get(
                    'gesture', 'positive_gestures').split(',')]

        pub, msg_class = rostopic.create_publisher(
            '/blender_api/available_emotion_states',
            'blender_api_msgs/AvailableEmotionStates', True)
        pub.publish(msg_class(['happy']))

        queue = MessageQueue()
        queue.subscribe('/blender_api/set_emotion_state', EmotionState)
        queue.subscribe('/blender_api/set_gesture', SetGesture)
        time.sleep(1)

        self.behavior_switch('btree_on')
        pub, msg_class = rostopic.create_publisher(
            '/camera/face_event', 'pi_face_tracker/FaceEvent', True)
        pub.publish(msg_class('new_face', 1))

        time.sleep(20)
        self.assertTrue(not queue.queue.empty())
        emotions = []
        gestures = []
        while not queue.queue.empty():
            msg = queue.get(timeout=1)
            if isinstance(msg, EmotionState):
                emotions.append(msg.name)
            elif isinstance(msg, SetGesture):
                gestures.append(msg.name)

        self.assertTrue(set(['happy']) | set(emotions))
        self.assertTrue(set(positive_gestures) & set(gestures))

    @unittest.skip("Not stable")
    def test_face_interaction2(self):
        new_arrival_emotions = [
            x.strip() for x in self.behavior_config.get(
                    'emotion', 'new_arrival_emotions').split(',')]
        positive_gestures = [
            x.strip() for x in self.behavior_config.get(
                    'gesture', 'positive_gestures').split(',')]
        queue = MessageQueue()
        queue.subscribe('/blender_api/set_emotion_state', EmotionState)
        queue.subscribe('/blender_api/set_gesture', SetGesture)
        time.sleep(1)

        self.behavior_switch('btree_on')
        pub, msg_class = rostopic.create_publisher(
            '/camera/face_event', 'pi_face_tracker/FaceEvent', True)
        pub.publish(msg_class('new_face', 1))

        time.sleep(20)
        self.assertTrue(not queue.queue.empty())
        emotions = []
        gestures = []
        while not queue.queue.empty():
            msg = queue.get(timeout=1)
            if isinstance(msg, EmotionState):
                emotions.append(msg.name)
            elif isinstance(msg, SetGesture):
                gestures.append(msg.name)

        self.assertTrue(set(new_arrival_emotions) | set(emotions))
        self.assertTrue(set(positive_gestures) & set(gestures))

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'eva_behavior', EvaBehaviorTest)

