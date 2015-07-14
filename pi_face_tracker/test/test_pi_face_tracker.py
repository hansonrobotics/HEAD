#!/usr/bin/env python

import unittest
import rospy
import os
import sys
import time

import rospkg
import roslaunch
from testing_tools import rosbag_msg_generator, play_rosbag
from pi_face_tracker.msg import FaceEvent, Faces

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

        self.test_data_path = '%s/test_data' % CWD

    def tearDown(self):
        self.runner.stop()

    def test_face_out(self, ):
        bag_file = '%s/face_out.bag' % self.test_data_path
        job = play_rosbag([bag_file, '-q'])

        face_detected = False
        face_location_msg = None
        face_event_msg = None
        timeout = 2
        while job.is_alive():
            try:
                face_location_msg = rospy.wait_for_message(
                    '/camera/face_locations', Faces, timeout)
            except rospy.exceptions.ROSException as e:
                rospy.loginfo(e)
                face_location_msg = None
            if face_location_msg and face_location_msg.faces:
                self.assertEqual(len(face_location_msg.faces), 1)
                face = face_location_msg.faces[0]
                self.assertGreater(face.attention, 0.99)

            try:
                face_event_msg = rospy.wait_for_message(
                    '/camera/face_event', FaceEvent, timeout)
            except rospy.exceptions.ROSException as e:
                rospy.loginfo(e)
                face_event_msg = None
            if face_event_msg:
                if not face_detected:
                    self.assertEqual(face_event_msg.face_event, 'new_face')
                    face_detected = True
                else:
                    self.assertEqual(face_event_msg.face_event, 'lost_face')
                    face_detected = False
        job.join()

    def test_face_in(self):
        bag_file = '%s/face_in.bag' % self.test_data_path
        job = play_rosbag(bag_file)

        face_detected = False
        face_location_msg = None
        face_event_msg = None
        timeout = 2
        while job.is_alive():
            try:
                face_location_msg = rospy.wait_for_message(
                    '/camera/face_locations', Faces, timeout)
            except rospy.exceptions.ROSException as e:
                rospy.loginfo(e)
                face_location_msg = None
            if face_location_msg and face_location_msg.faces:
                self.assertEqual(len(face_location_msg.faces), 1)
                face = face_location_msg.faces[0]
                self.assertGreater(face.attention, 0.99)

            try:
                face_event_msg = rospy.wait_for_message(
                    '/camera/face_event', FaceEvent, timeout)
            except rospy.exceptions.ROSException as e:
                rospy.loginfo(e)
                face_event_msg = None
            if face_event_msg:
                if not face_detected:
                    self.assertEqual(face_event_msg.face_event, 'new_face')
                    face_detected = True
                else:
                    self.assertEqual(face_event_msg.face_event, 'lost_face')
                    face_detected = False
        job.join()

    def test_noface(self):
        bag_file = '%s/noface.bag' % self.test_data_path
        job = play_rosbag([bag_file, '-q'])

        face_location_msg = None
        face_event_msg = None
        timeout = 2
        while job.is_alive():
            try:
                face_location_msg = rospy.wait_for_message(
                    '/camera/face_locations', Faces, timeout)
            except rospy.exceptions.ROSException as e:
                rospy.loginfo(e)
                face_location_msg = None
            self.assertIsNone(face_location_msg)

            try:
                face_event_msg = rospy.wait_for_message(
                    '/camera/face_event', FaceEvent, timeout)
            except rospy.exceptions.ROSException as e:
                rospy.loginfo(e)
                face_event_msg = None
            self.assertIsNone(face_event_msg)

        job.join()

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'pi_face_tracker', PiFaceTrackerTest)
    #unittest.main()

