#!/usr/bin/env python

import unittest
import os
import sys
import time
import shutil
import glob
import subprocess
from collections import Counter

import rospy
import roslaunch
import rosnode
import rostopic
import rospkg
from roslaunch import core
from roslaunch import nodeprocess
nodeprocess._TIMEOUT_SIGINT = 2
nodeprocess._TIMEOUT_SIGTERM = 1

from blender_api_msgs.msg import EmotionState
from pau2motors.msg import pau
from std_msgs.msg import String, Float64
from ros_pololu.msg import MotorCommand
from testing_tools import MessageQueue, play_rosbag, get_rosbag_file, rosbag_msg_generator

CWD = os.path.abspath(os.path.dirname(__file__))
PKG = 'robots_config'

class SophiaTest(unittest.TestCase):

    def setUp(self):
        self.run_id = 'test_robot'

        launch_file = '%s/sophia.launch' % CWD
        config = roslaunch.config.load_config_default([launch_file], None)

        self.socat_procs = []
        # virtual serial port
        serial_ports = {
            '/sophia/pololu_head/port_name': ('pololu0', 'pololu1'),
            '/sophia/dynamixel_manager/serial_ports/default/port_name': ('dynamixel0', 'dynamixel1')
        }
        for name, ports in serial_ports.items():
            port0 = '%s/%s' % (CWD, ports[0])
            port1 = '%s/%s' % (CWD, ports[1])
            cmd = 'socat -d -d pty,link=%s,raw,echo=0 pty,link=%s,raw,echo=0' % (port0, port1)
            proc = subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid)
            self.socat_procs.append(proc)
            config.add_param(core.Param(name, port0))

        self.runner = roslaunch.launch.ROSLaunchRunner(
            self.run_id, config, is_rostest=True)

    def tearDown(self):
        self.runner.stop()
        for proc in self.socat_procs:
            os.killpg(proc.pid, 2)

    def test_ros_message(self):
        self.runner.config.nodes = filter(
            lambda node: node.name != 'blender_api', self.runner.config.nodes)
        self.runner.launch()

        if not self.run_id in rosnode.get_node_names():
            rospy.init_node(self.run_id)

        os.system('rosservice call /sophia/head_pau_mux/select \"topic: \'/blender_api/get_pau\'\"')
        os.system('rosservice call /sophia/neck_pau_mux/select \"topic: \'/blender_api/get_pau\'\"')
        pau_messages = MessageQueue()
        pau_messages.subscribe('/blender_api/get_pau', pau)

        topics = [
            '/sophia/Frown_L_controller/command',
            '/sophia/Frown_R_controller/command',
            '/sophia/Jaw_Up_Down_controller/command',
            '/sophia/Lower_Gimbal_L_controller/command',
            '/sophia/Lower_Gimbal_R_controller/command',
            '/sophia/Neck_Rotation_controller/command',
            '/sophia/Smile_L_controller/command',
            '/sophia/Smile_R_controller/command',
            '/sophia/Upper_Gimbal_L_controller/command',
            '/sophia/Upper_Gimbal_R_controller/command',
        ]
        topic_queues = [MessageQueue() for topic in topics]
        [queue.subscribe(topic, Float64) for topic, queue in zip(topics, topic_queues)]

        head_queue = MessageQueue()
        head_queue.subscribe('/sophia/head/command', MotorCommand)
        time.sleep(5)

        #job = play_rosbag([get_rosbag_file('all_gestures')])
        #job.join()
        count = 3
        for topic, msg, timestamp in rosbag_msg_generator(get_rosbag_file('all_gestures')):
            print timestamp, topic
            count -= 1
            time.sleep(0.05)
            if count == 0: break

        msg_count = pau_messages.queue.qsize()
        print 'pau msgs %s' % msg_count

        # for queue in topic_queues:
        #     while not queue.queue.empty():
        #         msg = queue.get(10)
        #         print msg

        counter = Counter()
        while not head_queue.queue.empty():
            msg = head_queue.queue.get(1)
            counter[msg.joint_name] += 1

        for topic, queue in zip(topics, topic_queues):
            print topic, queue.queue.qsize()

        for k, v in counter.iteritems():
            print k, v


    def test_blender_ros_message(self):
        self.runner.launch()
        time.sleep(5) # wait for blender
        os.system('rosservice call /sophia/head_pau_mux/select \"topic: \'/blender_api/get_pau\'\"')
        os.system('rosservice call /sophia/neck_pau_mux/select \"topic: \'/blender_api/get_pau\'\"')
        pau_messages = MessageQueue()
        pau_messages.subscribe('/blender_api/get_pau', pau)

        topics = [
            '/sophia/Frown_L_controller/command',
            '/sophia/Frown_R_controller/command',
            '/sophia/Jaw_Up_Down_controller/command',
            '/sophia/Lower_Gimbal_L_controller/command',
            '/sophia/Lower_Gimbal_R_controller/command',
            '/sophia/Neck_Rotation_controller/command',
            '/sophia/Smile_L_controller/command',
            '/sophia/Smile_R_controller/command',
            '/sophia/Upper_Gimbal_L_controller/command',
            '/sophia/Upper_Gimbal_R_controller/command',
        ]
        topic_queues = [MessageQueue() for topic in topics]
        [queue.subscribe(topic, Float64) for topic, queue in zip(topics, topic_queues)]

        head_queue = MessageQueue()
        head_queue.subscribe('/sophia/head/command', MotorCommand)

        pub, msg_class = rostopic.create_publisher('/blender_api/set_gesture', 'blender_api_msgs/SetGesture', True)
        pub.publish(msg_class('blink', 1, 1, 1))

        os.system('rosservice call /sophia/head_pau_mux/select \"topic: \'/sophia/no_pau\'\"')
        os.system('rosservice call /sophia/neck_pau_mux/select \"topic: \'/sophia/cmd_neck_pau\'\"')
        time.sleep(2)

        counter = Counter()
        while not head_queue.queue.empty():
            msg = head_queue.queue.get(1)
            counter[msg.joint_name] += 1

        for topic, queue in zip(topics, topic_queues):
            print topic, queue.queue.qsize()

        for k, v in counter.iteritems():
            print k, v

if __name__ == '__main__':
    #import rostest
    #rostest.rosrun(PKG, 'test_sophia', SophiaTest)
    unittest.main()

