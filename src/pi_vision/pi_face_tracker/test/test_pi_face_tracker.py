#!/usr/bin/env python

import unittest
import rospy
import os
import sys
import time

import rospkg
import roslaunch
from testing_tools.misc import (rosbag_msg_generator, play_rosbag,
                        get_rosbag_file, wait_for_message, wait_for_messages)
from pi_face_tracker.msg import FaceEvent, Faces

from roslaunch import nodeprocess
nodeprocess._TIMEOUT_SIGINT = 2
nodeprocess._TIMEOUT_SIGTERM = 1

CWD = os.path.abspath(os.path.dirname(__file__))
PKG = 'pi_face_tracker'

class PiFaceTrackerTest(unittest.TestCase):

    def setUp(self):
        rospack = rospkg.RosPack()
        config_file = os.path.join(
            rospack.get_path(PKG), 'launch', 'face_tracker_usb_cam.launch')
        if os.path.isfile(config_file):
            config = roslaunch.config.load_config_default([config_file], None)
            self.runner = roslaunch.launch.ROSLaunchRunner(
                'test_pi_face_tracker', config)
            self.runner.launch()

    def tearDown(self):
        self.runner.stop()

    def test_noface(self):
        bag_file = get_rosbag_file('empty')
        job = play_rosbag([bag_file, '-q'])
        face_event_msg, face_location_msg = None, None
        fps = 25
        wait = 1.0/fps
        while job.is_alive():
            face_location_msg, face_event_msg = wait_for_messages(
                ['/camera/face_locations', '/camera/face_event'],
                [Faces, FaceEvent], wait)
            self.assertIsNone(face_location_msg)
            self.assertIsNone(face_event_msg)

        job.join()

    def test_face_in_out(self):
        face_event_msg, face_location_msg = None, None
        fps = 25
        wait = 1.0/fps
        for _, _, _ in rosbag_msg_generator(get_rosbag_file('face')):
            face_location_msg, face_event_msg = wait_for_messages(
                ['/camera/face_locations', '/camera/face_event'],
                [Faces, FaceEvent], wait)
            if face_location_msg and face_location_msg.faces:
                self.assertEqual(len(face_location_msg.faces), 1)
                face = face_location_msg.faces[0]
                self.assertGreater(face.attention, 0.99)
            if face_event_msg:
                self.assertEqual(face_event_msg.face_event, 'new_face')

        for _, _, _ in rosbag_msg_generator(get_rosbag_file('empty')):
            face_event_msg = wait_for_message(
                '/camera/face_event', FaceEvent, wait)
            if face_event_msg:
                self.assertEqual(face_event_msg.face_event, 'lost_face')


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'pi_face_tracker', PiFaceTrackerTest)

