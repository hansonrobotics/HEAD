#!/usr/bin/env python

import unittest
import os
import sys
import time
import ConfigParser
import shutil
import glob

import rospy
import roslaunch
import rosnode
import rostopic
import rospkg
from roslaunch import core
from roslaunch import nodeprocess
nodeprocess._TIMEOUT_SIGINT = 2
nodeprocess._TIMEOUT_SIGTERM = 1

from pi_face_tracker.msg import FaceEvent
from blender_api_msgs.msg import EmotionState
from std_msgs.msg import String
from testing_tools import (wait_for, play_rosbag, create_msg_listener,
                            capture_screen, capture_camera, startxvfb, stopxvfb,
                            get_rosbag_file, MessageQueue)

CWD = os.path.abspath(os.path.dirname(__file__))
PKG = 'robots_config'

class RobotTest(unittest.TestCase):

    def setUp(self):
        self.run_id = 'test_robot'
        rospack = rospkg.RosPack()
        config = roslaunch.config.ROSLaunchConfig()
        self.test_data_path = '%s/test_data' % CWD
        tts_path = rospack.get_path('tts')
        self.tts_output = os.path.join(tts_path, 'tmp')
        files = glob.glob('%s/*.wav' % self.tts_output)
        if files:
            shutil.rmtree('%s.bak' % self.tts_output)
            shutil.move(self.tts_output, '%s.bak' % self.tts_output)
            os.makedirs(self.tts_output)

        self.output_video = '%s/output_video' % CWD
        if not os.path.isdir(self.output_video):
            os.makedirs(self.output_video)
        self.output_audio = '%s/output_audio' % CWD
        if not os.path.isdir(self.output_audio):
            os.makedirs(self.output_audio)

        # blender_api
        blender_api_path = os.path.join(
            rospack.get_path('blender_api_msgs'), '../blender_api')
        config.add_node(
            core.Node(
                package='blender_api_msgs', node_type='blender',
                args='-y %s/Eva.blend -P %s/autostart.py' % (
                        blender_api_path, blender_api_path),
                name='blender_api')
            )

        # eva_behavior
        eva_behavior_path = rospack.get_path('eva_behavior')
        self.behavior_config = ConfigParser.ConfigParser()
        config_file = os.path.join(eva_behavior_path, 'behavior.cfg')
        self.behavior_config.read(config_file)
        config.add_node(
            core.Node(
                package='eva_behavior', node_type='main.py',
                name='Eva_behavior')
            )
        
        # pi_face_tracker
        config_file = os.path.join(
            rospack.get_path('pi_face_tracker'), 'launch',
            'face_tracker_usb_cam.launch')
        loader = roslaunch.xmlloader.XmlLoader()
        loader.load(config_file, config)

        # tts
        config.add_node(
            core.Node(
                package='tts', node_type='tts_talker.py',
                name='tts')
            )

        # chatbot
        config.add_node(
            core.Node(
                package='chatbot', node_type='ai.py',
                args='%s/../../chatbot/aiml' % CWD,
                name='chatbot_ai')
            )

        config.add_param(core.Param('/tts/topic_name', 'chatbot_responses'))
        self.runner = roslaunch.launch.ROSLaunchRunner(
            self.run_id, config, is_rostest=True)

        self.display = os.environ.get('DISPLAY', ':0')
        if not self.display == ':0':
            startxvfb(self.display)
        self.runner.launch()

        for node in config.nodes:
            wait_for('%s/%s' % (node.namespace, node.name))

        time.sleep(15) # Wait for blender rendering done and chatbot

    def tearDown(self):
        self.runner.stop()
        if not self.display == ':0':
            stopxvfb(self.display)

    def test(self):
        new_arrival_emotions = [
            x.strip() for x in self.behavior_config.get(
                    'emotion', 'new_arrival_emotions').split(',')]
        pub, msg_class = rostopic.create_publisher(
            '/behavior_switch', 'std_msgs/String', True)
        pub.publish(msg_class('btree_on'))
        bag_file = get_rosbag_file('face')

        emo_msg_listener = create_msg_listener(
            '/blender_api/set_emotion_state', EmotionState, 10)
        emo_msg_listener.start()
        cam_output = '%s/cam_new_arrival_emotion.avi' % \
                        self.output_video
        screen_output = '%s/screen_new_arrival_emotion.avi' % \
                        self.output_video
        duration = 5
        with capture_camera(cam_output, duration):
            with capture_screen(screen_output, duration):
                job = play_rosbag([bag_file, '-q'])
        job.join()
        emo_msg = emo_msg_listener.join()

        self.assertIn(emo_msg.name, new_arrival_emotions)

    def test_chat(self):
        import re
        r = re.compile('[\W_]+')
        pub, msg_class = rostopic.create_publisher(
            '/chatbot_speech', 'chatbot/ChatMessage', True)
        words = ['Hi', 'How are you', 'What\'s your name']
        duration = 5
        queue = MessageQueue()
        queue.subscribe('/chatbot_responses', String)
        for word in words:
            cam_output = '%s/cam_%s.avi' % (
                self.output_video, r.sub('', word))
            screen_output = '%s/screen_%s.avi' % (
                self.output_video, r.sub('',word))
            with capture_camera(cam_output, duration):
                with capture_screen(screen_output, duration):
                    pub.publish(msg_class(word, 100))
            msg = queue.get()
            cam_output_new = '%s/cam_%s_%s.avi' % (
                self.output_video, r.sub('', word), r.sub('', msg.data))
            shutil.move(cam_output, cam_output_new)
            screen_output_new = '%s/screen_%s_%s.avi' % (
                self.output_video, r.sub('', word), r.sub('', msg.data))
            shutil.move(screen_output, screen_output_new)
            files = glob.glob('%s/*.wav' % self.tts_output)
            self.assertEqual(len(files), 1)
            shutil.move(
                files[0], '%s/%s.wav' % (self.output_audio, r.sub('', msg.data)))

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_robot', RobotTest)

