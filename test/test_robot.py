#!/usr/bin/env python

import unittest
import os
import sys
import time

import rospy
import roslaunch
import rosnode
import rostopic
import rospkg
from roslaunch import core
from roslaunch import nodeprocess
nodeprocess._TIMEOUT_SIGINT = 2
nodeprocess._TIMEOUT_SIGTERM = 1

from testing_tools import wait_for

PKG = 'robots_config'

class RobotTest(unittest.TestCase):

    def setUp(self):
        self.run_id = 'test_robot'
        rospack = rospkg.RosPack()
        config = roslaunch.config.ROSLaunchConfig()

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
        config.add_node(
            core.Node(
                package='usb_cam', node_type='usb_cam_node',
                name='camera')
            )
        
        self.runner = roslaunch.launch.ROSLaunchRunner(
            self.run_id, config, is_rostest=True)
        self.runner.launch()

        for node in config.nodes:
            wait_for('%s/%s' % (node.namespace, node.name))

    def tearDown(self):
        self.runner.stop()

    def test(self):
        pub, msg_class = rostopic.create_publisher(
            '/behavior_switch', 'std_msgs/String', True)
        pub.publish(msg_class('btree_on'))

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_robot', RobotTest)
    #unittest.main()

