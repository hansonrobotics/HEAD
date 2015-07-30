#!/usr/bin/env python

import unittest
import rospy
import os
import sys
import time

import rospkg
import roslaunch
import rostopic
from roslaunch import core
from testing_tools import (wait_for, wait_for_message, wait_for_messages,
                            startxvfb, stopxvfb, capture_screen, run_shell_cmd)
from blender_api_msgs.msg import *
from genpy import Duration

from roslaunch import nodeprocess
nodeprocess._TIMEOUT_SIGINT = 2
nodeprocess._TIMEOUT_SIGTERM = 1

CWD = os.path.abspath(os.path.dirname(__file__))
PKG = 'blender_api'

def parse_msg(msg):
    return eval(msg.split(':')[1].strip())

class BlenderAPITest(unittest.TestCase):

    def setUp(self):
        self.run_id = 'test_blender_api'
        rospack = rospkg.RosPack()
        config = roslaunch.config.ROSLaunchConfig()
        self.output_dir = 'output_video'
        if not os.path.isdir(self.output_dir):
            os.makedirs(self.output_dir)

        blender_api_path = rospack.get_path('blender_api')
        config.add_node(
            core.Node(
                package='blender_api', node_type='blender',
                args='-y %s/Eva.blend -P %s/autostart.py' % (
                        blender_api_path, blender_api_path),
                name='blender_api')
            )
        self.runner = roslaunch.launch.ROSLaunchRunner(
            self.run_id, config, is_rostest=True)

        self.display = os.environ.get('DISPLAY', ':0')
        startxvfb(self.display, '1920x1080x24')
        self.runner.launch()

        for node in config.nodes:
            wait_for('%s/%s' % (node.namespace, node.name))

        time.sleep(5) # Wait for blender rendering done

    def tearDown(self):
        self.runner.stop()
        stopxvfb(self.display)

    def test_emotion_state(self):
        available_emotions = parse_msg(run_shell_cmd(
            'rostopic echo -n1 /blender_api/available_emotion_states', True))
        pub, msg_class = rostopic.create_publisher(
            '/blender_api/set_emotion_state',
            'blender_api_msgs/EmotionState', True)
        timeout = 2
        for emotion in available_emotions:
            with capture_screen('%s/%s.avi' % (self.output_dir, emotion), timeout):
                pub.publish(msg_class(emotion, 1, Duration(1, 0)))

    def test_gesture(self):
        available_gestures = parse_msg(run_shell_cmd(
            'rostopic echo -n1 /blender_api/available_gestures', True))
        pub, msg_class = rostopic.create_publisher(
            '/blender_api/set_gesture',
            'blender_api_msgs/SetGesture', True)
        timeout = 2
        for gesture in available_gestures:
            if gesture == 'all': continue
            with capture_screen('%s/%s.avi' % (self.output_dir, gesture), timeout):
                pub.publish(msg_class(gesture, 1, 1, 1))

    def test_viseme(self):
        available_visemes = parse_msg(run_shell_cmd(
            'rostopic echo -n1 /blender_api/available_visemes', True))
        pub, msg_class = rostopic.create_publisher(
            '/blender_api/queue_viseme',
            'blender_api_msgs/Viseme', True)
        timeout = 2
        for viseme in available_visemes:
            if 'old' in viseme: continue
            with capture_screen('%s/%s.avi' % (self.output_dir, viseme), timeout):
                pub.publish(msg_class(
                        viseme, Duration(0, 0), Duration(0, 5*1e8), 0.1, 0.8, 1))

    def test_gaze_target(self):
        pub, msg_class = rostopic.create_publisher(
            '/blender_api/set_gaze_target',
            'blender_api_msgs/Target', True)
        timeout = 2
        targets = {
            'gaze_center': (1,0,0),
            'gaze_right':(0,1,0),
            'gaze_left':(0,-1,0),
            'gaze_up':(1,0,0.5),
            'gaze_down':(1,0,-0.5)}
        with capture_screen('%s/gaze_target.avi' % self.output_dir, len(targets)*timeout):
            for name in ['gaze_right', 'gaze_up', 'gaze_left',
                        'gaze_down', 'gaze_center']:
                pub.publish(msg_class(*targets[name]))
                time.sleep(timeout)

    def test_face_target(self):
        pub, msg_class = rostopic.create_publisher(
            '/blender_api/set_face_target',
            'blender_api_msgs/Target', True)
        timeout = 2
        targets = {
            'face_center': (1,0,0),
            'face_right':(0,1,0),
            'face_left':(0,-1,0),
            'face_up':(1,0,0.5),
            'face_down':(1,0,-0.5)}
        with capture_screen('%s/face_target.avi' % self.output_dir, len(targets)*timeout):
            for name in ['face_right', 'face_up', 'face_left',
                        'face_down', 'face_center']:
                pub.publish(msg_class(*targets[name]))
                time.sleep(timeout)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'blender_api', BlenderAPITest)

