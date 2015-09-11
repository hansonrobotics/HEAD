#!/usr/bin/env python

import unittest
import os
import time
import subprocess
from collections import Counter

import roslaunch
import rosservice
from roslaunch import core
from roslaunch import nodeprocess
nodeprocess._TIMEOUT_SIGINT = 2
nodeprocess._TIMEOUT_SIGTERM = 1

from pau2motors.msg import pau
from std_msgs.msg import Float64
from ros_pololu.msg import MotorCommand
from testing_tools import MessageQueue

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

    def test_blender_ros_message(self):
        self.runner.launch()
        time.sleep(5) # wait for blender
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

        time.sleep(2) # wait for connection to synchronize

        # pause command listener
        request, response = rosservice.call_service('/blender_api/set_param', ["bpy.context.scene['commandListenerActive']", "False"])
        self.assertTrue(response)
        time.sleep(3) # wait for cached messages to be handled

        # reset the messages in these queues
        for queue in topic_queues + [pau_messages, head_queue]:
            queue.clear()

        # re-activate command listener
        request, response = rosservice.call_service('/blender_api/set_param', ["bpy.context.scene['commandListenerActive']", "True"])
        self.assertTrue(response)
        time.sleep(2)

        request, response = rosservice.call_service('/blender_api/set_param', ["bpy.context.scene['commandListenerActive']", "False"])
        self.assertTrue(response)
        time.sleep(3) # wait for cached messages to be handled

        num_pau_message = pau_messages.queue.qsize()
        self.assertTrue(num_pau_message>0)

        counter = Counter()
        while not head_queue.queue.empty():
            msg = head_queue.queue.get(1)
            counter[msg.joint_name] += 1

        for topic, queue in zip(topics, topic_queues):
            self.assertEqual(queue.queue.qsize(), num_pau_message)

        for k, v in counter.iteritems():
            self.assertEqual(v, num_pau_message)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_sophia', SophiaTest)

