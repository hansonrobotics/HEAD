#!/usr/bin/env python

import unittest
import rospy
import os
import sys
import yaml

import roslaunch
import rostopic
import roslib
from roslaunch import core

from testing_tools import wait_for, wait_for_message
from genpy.message import fill_message_args
from sensor_msgs.msg import JointState

from roslaunch import nodeprocess
nodeprocess._TIMEOUT_SIGINT = 2
nodeprocess._TIMEOUT_SIGTERM = 1

CWD = os.path.abspath(os.path.dirname(__file__))
PKG = 'perception'

class PerceptionTest(unittest.TestCase):

    def setUp(self):
        self.run_id = 'test_perception'
        config = roslaunch.config.ROSLaunchConfig()
        config.add_node(
            core.Node(
                package='perception', node_type='joint_state_publisher.py',
                name='state_publisher')
            )

        self.joints = ['Eyes_Pitch', 'Eye_L', 'Eye_R', 'roll_base_joint',
                    'pitch_base_joint', 'yaw_joint', 'roll_neck_joint',
                    'pitch_neck_joint']
        config.add_param(core.Param('/state_publisher/pau_joints',
                                    ';'.join(self.joints)))
        config.add_node(
            core.Node(
                package='perception', node_type='faces_tf2_broadcaster.py',
                name='faces_broadcaster')
            )
        self.runner = roslaunch.launch.ROSLaunchRunner(
            self.run_id, config, is_rostest=True)
        self.runner.launch()

        for node in config.nodes:
            wait_for('%s/%s' % (node.namespace, node.name))

    def tearDown(self):
        self.runner.stop()

    def check_msg(self, msg_str, expects):
        pub, msg_class = rostopic.create_publisher(
            '/blender_api/get_pau', 'pau2motors/pau', True)
        msg = msg_class()
        fill_message_args(msg, yaml.load(msg_str))
        pub.publish(msg)
        msg = wait_for_message('/joint_states_combined', JointState, 2)
        for key in ['position', 'name']:
            self.assertListEqual(list(getattr(msg, key)), expects[key])
        pub.unregister()

    def test_joint_states_combined(self):
        msg_str = """
m_headRotation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0
m_headTranslation:
    x: 0.0
    y: 0.0
    z: 0.0
m_neckRotation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
m_eyeGazeLeftPitch: 0.0
m_eyeGazeLeftYaw: 0.0
m_eyeGazeRightPitch: 0.0
m_eyeGazeRightYaw: 0.0
m_coeffs: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"""
        expects = {
            'name': self.joints,
            'position': [0, 0, 0, 0, 0, 0, 0, 0]
            }
        self.check_msg(msg_str, expects)

    def test_joint_states_combined2(self):
        msg_str = """
m_headRotation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0
m_headTranslation:
    x: 0.0
    y: 0.0
    z: 0.0
m_neckRotation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
m_eyeGazeLeftPitch: 1.0
m_eyeGazeLeftYaw: 0.0
m_eyeGazeRightPitch: 0.0
m_eyeGazeRightYaw: 0.0
m_coeffs: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"""
        expects = {
            'name': self.joints,
            'position': [1, 0, 0, 0, 0, 0, 0, 0]
            }
        self.check_msg(msg_str, expects)

    def test_joint_states_combined3(self):
        msg_str = """
m_headRotation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0
m_headTranslation:
    x: 0.0
    y: 0.0
    z: 0.0
m_neckRotation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
m_eyeGazeLeftPitch: 0.0
m_eyeGazeLeftYaw: 1.0
m_eyeGazeRightPitch: 0.0
m_eyeGazeRightYaw: 0.0
m_coeffs: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"""
        expects = {
            'name': self.joints,
            'position': [0, -1, -1, 0, 0, 0, 0, 0]
            }
        self.check_msg(msg_str, expects)

    def test_joint_states_combined4(self):
        msg_str = """
m_headRotation:
    x: 1.0
    y: 0.0
    z: 0.0
    w: 0.0
m_headTranslation:
    x: 0.0
    y: 0.0
    z: 0.0
m_neckRotation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
m_eyeGazeLeftPitch: 0.0
m_eyeGazeLeftYaw: 0.0
m_eyeGazeRightPitch: 0.0
m_eyeGazeRightYaw: 0.0
m_coeffs: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"""
        pitch_neck_joint = -1.5707963267948966
        expects = {
            'name': self.joints,
            'position': [0, 0, 0, 0, pitch_neck_joint, 0, 0, pitch_neck_joint]
            }
        self.check_msg(msg_str, expects)

    def test_joint_states_combined5(self):
        msg_str = """
m_headRotation:
    x: 1.0
    y: 0.0
    z: 0.0
    w: 1.0
m_headTranslation:
    x: 0.0
    y: 0.0
    z: 0.0
m_neckRotation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
m_eyeGazeLeftPitch: 0.0
m_eyeGazeLeftYaw: 0.0
m_eyeGazeRightPitch: 0.0
m_eyeGazeRightYaw: 0.0
m_coeffs: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"""
        pitch_neck_joint = 0.7853981633974483
        expects = {
            'name': self.joints,
            'position': [0, 0, 0, 0, pitch_neck_joint, 0, 0, pitch_neck_joint]
            }
        self.check_msg(msg_str, expects)
if __name__ == '__main__':
    #import rostest
    #rostest.rosrun(PKG, 'perception', PerceptionTest)
    unittest.main()
