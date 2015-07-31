#!/usr/bin/env python

import unittest
import os
import sys
import time
import ConfigParser

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
from testing_tools import (wait_for, play_rosbag, create_msg_listener,
                            capture_screen, capture_camera, startxvfb, stopxvfb,
                            get_rosbag_file)

CWD = os.path.abspath(os.path.dirname(__file__))
PKG = 'robots_config'

class RobotTest(unittest.TestCase):

    def setUp(self):
        self.run_id = 'test_robot'
        rospack = rospkg.RosPack()
        config = roslaunch.config.ROSLaunchConfig()
        self.test_data_path = '%s/test_data' % CWD
        self.output_data_path = '%s/output' % CWD
        if not os.path.isdir(self.output_data_path):
            os.makedirs(self.output_data_path)

        # blender_api
        blender_api_path = rospack.get_path('blender_api')
        config.add_node(
            core.Node(
                package='blender_api', node_type='blender',
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

        # camera
        # config.add_node(
        #     core.Node(
        #         package='usb_cam', node_type='usb_cam_node',
        #         name='camera')
        #     )

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
                        self.output_data_path
        screen_output = '%s/screen_new_arrival_emotion.avi' % \
                        self.output_data_path
        duration = 15
        with capture_camera(cam_output, duration):
            with capture_screen(screen_output, duration):
                job = play_rosbag([bag_file, '-q'])
        job.join()
        emo_msg = emo_msg_listener.join()

        self.assertIn(emo_msg.name, new_arrival_emotions)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_robot', RobotTest)

