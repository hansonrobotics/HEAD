#!/usr/bin/env python

import unittest
import rospy
import os
import sys
import time
import yaml
import threading

import rospkg
import roslaunch
import rosparam
from roslaunch import core

from pau2motors.msg import pau
from genpy.message import fill_message_args
from std_msgs.msg import Float64

from testing_tools import wait_for_message

CWD = os.path.abspath(os.path.dirname(__file__))
PKG = 'pau2motors'

class Pau2MotorsTest(unittest.TestCase):

    def setUp(self):
        self.run_id = 'test_pau2motors'
        self.is_alive = True
        self.motors = [
            'Lower_Gimbal_L_controller',
            'Lower_Gimbal_R_controller',
            'Upper_Gimbal_L_controller',
            'Upper_Gimbal_R_controller',
            'Neck_Rotation_controller',
            ]

    def tearDown(self):
        if hasattr(self, 'runner'):
            self.runner.stop()

    def keep_running(self, func, args, rate=0.5):
        while self.is_alive:
            try:
                func(args)
                time.sleep(rate)
            except KeyboardInterrupt:
                break

    def _init_neck_upper(self):
        motors_config = '%s/neck-upper.yaml' % CWD
        config = roslaunch.config.ROSLaunchConfig()
        config.add_node(
            core.Node(
                package='pau2motors', node_type='pau2motors_node.py',
                name='pau2motors')
            )
        config.add_executable(
            core.Executable(
                cmd='rosparam load', args=(motors_config,),
                phase=core.PHASE_SETUP)
            )
        self.runner = roslaunch.launch.ROSLaunchRunner(
            self.run_id, config)
        self.runner.launch()
        rospy.init_node('test_pau2motors')

    def check_msg(self, msg_str, expects):
        self._init_neck_upper()
        pub = rospy.Publisher('/neck_pau',
            pau, queue_size=30, latch=True)
        msg_args = yaml.load(msg_str)
        msg = pau()
        fill_message_args(msg, msg_args)

        t = threading.Thread(
            target=self.keep_running, args=(pub.publish, msg))
        t.daemon = True
        t.start()

        timeout = 2
        for motor in self.motors:
            msg = wait_for_message('/%s/command' % motor, Float64, timeout)
            while msg is None:
                msg = wait_for_message('/%s/command' % motor, Float64, timeout)
            self.assertAlmostEqual(msg.data, expects[motor])
            #print >> sys.stderr, "'%s': %s" % (motor, msg.data)

        self.is_alive = False
        t.join()
        pub.unregister()

    def test_neck_upper1(self):
        msg_str = """
m_headRotation:
    x: 0.0
    y: 0.085
    z: 0.0
    w: 0.9963809512430474
m_headTranslation:
    x: 0.0
    y: 0.0
    z: 0.0
m_neckRotation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
m_eyeGazeLeftPitch: -0.0216188617051
m_eyeGazeLeftYaw: -0.00167124893051
m_eyeGazeRightPitch: -0.0209033191204
m_eyeGazeRightYaw: -0.0445618629456
m_coeffs: [0.0, 0.75, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.6212145090103149, 0.26078522205352783, 0.6212085485458374,
0.29769060015678406, 0.0, 0.0, 0.0, 0.0, 0.0075283292680978775, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0]"""
        expects = {
            'Lower_Gimbal_L_controller':0.0,
            'Lower_Gimbal_R_controller':0.0,
            'Upper_Gimbal_L_controller':0.380082675581,
            'Upper_Gimbal_R_controller':0.382519581875,
            'Neck_Rotation_controller':0.0,
            }
        self.check_msg(msg_str, expects)

    def test_neck_upper2(self):
        msg_str = """
m_headRotation:
  x: 0.0
  y: -0.085
  z: 0.0
  w: 0.9963809512430474
m_headTranslation:
  x: 0.0
  y: 0.0
  z: 0.0
m_neckRotation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0
m_eyeGazeLeftPitch: -0.0216188617051
m_eyeGazeLeftYaw: -0.00167124893051
m_eyeGazeRightPitch: -0.0209033191204
m_eyeGazeRightYaw: -0.0445618629456
m_coeffs: [0.0, 0.75, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.6212145090103149, 0.26078522205352783, 0.6212085485458374,
0.29769060015678406, 0.0, 0.0, 0.0, 0.0, 0.0075283292680978775, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0]"""
        expects = {
            'Lower_Gimbal_L_controller': 0.0,
            'Lower_Gimbal_R_controller': 0.0,
            'Upper_Gimbal_L_controller': -0.382519581875,
            'Upper_Gimbal_R_controller': -0.380082675581,
            'Neck_Rotation_controller': 0.0,
            }

        self.check_msg(msg_str, expects)

    def test_neck_upper3(self):
        msg_str = """
m_headRotation:
  x: 0.0
  y: 0.085
  z: 0.0
  w: 0.9963809512430474
m_headTranslation:
  x: 0.0
  y: 0.0
  z: 0.0
m_neckRotation:
  x: 0.0
  y: -0.085
  z: 0.0
  w: 0.9963809512430474
m_eyeGazeLeftPitch: -0.0216188617051
m_eyeGazeLeftYaw: -0.00167124893051
m_eyeGazeRightPitch: -0.0209033191204
m_eyeGazeRightYaw: -0.0445618629456
m_coeffs: [0.0, 0.75, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.6212145090103149, 0.26078522205352783, 0.6212085485458374,
0.29769060015678406, 0.0, 0.0, 0.0, 0.0, 0.0075283292680978775, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0]"""
        expects = {
            'Lower_Gimbal_L_controller': 0.0,
            'Lower_Gimbal_R_controller': 0.0,
            'Upper_Gimbal_L_controller': 0.380082675581,
            'Upper_Gimbal_R_controller': 0.382519581875,
            'Neck_Rotation_controller': 0.0,
            }
        self.check_msg(msg_str, expects)

    def test_neck_upper4(self):
        msg_str = """
m_headRotation:
  x: 0.0
  y: -0.085
  z: 0.0
  w: 0.9963809512430474
m_headTranslation:
  x: 0.0
  y: 0.0
  z: 0.0
m_neckRotation:
  x: 0.0
  y: 0.085
  z: 0.0
  w: 0.9963809512430474
m_eyeGazeLeftPitch: -0.0216188617051
m_eyeGazeLeftYaw: -0.00167124893051
m_eyeGazeRightPitch: -0.0209033191204
m_eyeGazeRightYaw: -0.0445618629456
m_coeffs: [0.0, 0.75, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.6212145090103149, 0.26078522205352783, 0.6212085485458374,
0.29769060015678406, 0.0, 0.0, 0.0, 0.0, 0.0075283292680978775, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0]"""
        expects = {
            'Lower_Gimbal_L_controller': 0.0,
            'Lower_Gimbal_R_controller': 0.0,
            'Upper_Gimbal_L_controller': -0.382519581875,
            'Upper_Gimbal_R_controller': -0.380082675581,
            'Neck_Rotation_controller': 0.0,
            }
        self.check_msg(msg_str, expects)

    def test_neck_upper5(self):
        msg_str = """
m_headRotation:
  x: 0.0
  y: -0.085
  z: 0.0
  w: 0.9963809512430474
m_headTranslation:
  x: 0.0
  y: 0.0
  z: 0.0
m_neckRotation:
  x: 0.0
  y: -0.085
  z: 0.0
  w: 0.9963809512430474
m_eyeGazeLeftPitch: -0.0216188617051
m_eyeGazeLeftYaw: -0.00167124893051
m_eyeGazeRightPitch: -0.0209033191204
m_eyeGazeRightYaw: -0.0445618629456
m_coeffs: [0.0, 0.75, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.6212145090103149, 0.26078522205352783, 0.6212085485458374,
0.29769060015678406, 0.0, 0.0, 0.0, 0.0, 0.0075283292680978775, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0]"""
        expects = {
            'Lower_Gimbal_L_controller': 0.0,
            'Lower_Gimbal_R_controller': 0.0,
            'Upper_Gimbal_L_controller': -0.382519581875,
            'Upper_Gimbal_R_controller': -0.380082675581,
            'Neck_Rotation_controller': 0.0,
            }
        self.check_msg(msg_str, expects)

    def test_neck_upper6(self):
        msg_str = """
m_headRotation:
  x: 0.0
  y: 0.085
  z: 0.0
  w: 0.9963809512430474
m_headTranslation:
  x: 0.0
  y: 0.0
  z: 0.0
m_neckRotation:
  x: 0.0
  y: 0.085
  z: 0.0
  w: 0.9963809512430474
m_eyeGazeLeftPitch: -0.0216188617051
m_eyeGazeLeftYaw: -0.00167124893051
m_eyeGazeRightPitch: -0.0209033191204
m_eyeGazeRightYaw: -0.0445618629456
m_coeffs: [0.0, 0.75, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.6212145090103149, 0.26078522205352783, 0.6212085485458374,
0.29769060015678406, 0.0, 0.0, 0.0, 0.0, 0.0075283292680978775, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0]"""
        expects = {
            'Lower_Gimbal_L_controller': 0.0,
            'Lower_Gimbal_R_controller': 0.0,
            'Upper_Gimbal_L_controller': 0.380082675581,
            'Upper_Gimbal_R_controller': 0.382519581875,
            'Neck_Rotation_controller': 0.0,
            }
        self.check_msg(msg_str, expects)

    def test_neck_upper7(self):
        msg_str = """
m_headRotation:
  x: 0.0
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
m_eyeGazeLeftPitch: -0.0216188617051
m_eyeGazeLeftYaw: -0.00167124893051
m_eyeGazeRightPitch: -0.0209033191204
m_eyeGazeRightYaw: -0.0445618629456
m_coeffs: [0.0, 0.75, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.6212145090103149, 0.26078522205352783, 0.6212085485458374,
0.29769060015678406, 0.0, 0.0, 0.0, 0.0, 0.0075283292680978775, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0]"""
        expects = {
                'Lower_Gimbal_L_controller': 0.0,
                'Lower_Gimbal_R_controller': 0.0,
                'Upper_Gimbal_L_controller': 0.0,
                'Upper_Gimbal_R_controller': 0.0,
                'Neck_Rotation_controller': 0.0,
            }
        self.check_msg(msg_str, expects)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'pau2motors', Pau2MotorsTest)
    #unittest.main()

