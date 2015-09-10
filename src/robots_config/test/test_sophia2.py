#!/usr/bin/env python

import unittest
import os
import time
import subprocess
from collections import Counter

import roslaunch
import rostopic
import rosparam
from roslaunch import core
from roslaunch import nodeprocess
nodeprocess._TIMEOUT_SIGINT = 2
nodeprocess._TIMEOUT_SIGTERM = 1

from pau2motors.msg import pau
from std_msgs.msg import Float64
from ros_pololu.msg import MotorCommand
from testing_tools import MessageQueue, get_rosbag_file, rosbag_msg_generator, PololuSerialReader

CWD = os.path.abspath(os.path.dirname(__file__))
PKG = 'robots_config'

class SophiaTest2(unittest.TestCase):

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

        pau_messages = MessageQueue()
        pau_messages.subscribe('/blender_api/get_pau', pau)
        head_pau_messages = MessageQueue()
        head_pau_messages.subscribe('/sophia/head_pau', pau)
        neck_pau_messages = MessageQueue()
        neck_pau_messages.subscribe('/sophia/neck_pau', pau)

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
        time.sleep(1)

        # send N messages for connections to synchronize
        # we may need to investigate why the first few messages are eaten up
        count = 10
        for topic, msg, timestamp in rosbag_msg_generator(
                                    get_rosbag_file('all_gestures')):
            count -= 1
            time.sleep(0.1)
            if count == 0: break

        # reset the messages in these queues
        for queue in topic_queues + \
            [head_pau_messages, neck_pau_messages, pau_messages, head_queue]:
            queue.clear()

        count = 100
        for topic, msg, timestamp in rosbag_msg_generator(
                                    get_rosbag_file('all_gestures')):
            count -= 1
            time.sleep(0.02)
            if count == 0: break

        print 'pau %s, head pau %s, neck pau %s' % (
            pau_messages.queue.qsize(), head_pau_messages.queue.qsize(),
            neck_pau_messages.queue.qsize())
        time.sleep(2)

        num_pau_message = pau_messages.queue.qsize()
        self.assertEqual(num_pau_message, head_pau_messages.queue.qsize())
        self.assertEqual(num_pau_message, neck_pau_messages.queue.qsize())

        counter = Counter()
        while not head_queue.queue.empty():
            msg = head_queue.queue.get(1)
            counter[msg.joint_name] += 1

        for topic, queue in zip(topics, topic_queues):
            self.assertEqual(queue.queue.qsize(), num_pau_message)

        for k, v in counter.iteritems():
            self.assertEqual(v, num_pau_message)


    def test_head_pau_message(self):
        reader = PololuSerialReader('%s/pololu1' % CWD)
        self.runner.config.nodes = filter(
            lambda node: node.name != 'blender_api', self.runner.config.nodes)
        self.runner.launch()
        time.sleep(2) # wait for pau2motors

        head_pau_motors = rosparam.get_param('/sophia/pau2motors/topics')['head_pau'].split(';')
        motors = rosparam.get_param('/sophia/motors')
        pololu_motor_config = {}
        for motor in motors:
            if 'motor_id' in motor:
                pololu_motor_config[motor['motor_id']] = motor

        self.assertTrue(len(head_pau_motors)>0)

        head_pau_messages = MessageQueue()
        head_pau_messages.subscribe('/sophia/head_pau', pau)

        head_queue = MessageQueue()
        head_queue.subscribe('/sophia/head/command', MotorCommand)
        time.sleep(1)

        pau_message = pau()
        pub, msg_class = rostopic.create_publisher('/sophia/head_pau', 'pau2motors/pau', True)
        pub.publish(msg_class(m_coeffs=[0]*48))
        time.sleep(2)

        print 'head pau %s' % head_pau_messages.queue.qsize()
        for _ in range(len(head_pau_motors)*3):
            motor_id, cmd, value = reader.read()
            config = pololu_motor_config[motor_id]
            if cmd == 'position':
                print motor_id, cmd, value
                if config['name'] in ['Cheek-Squint']: continue # these motors are exceptions
                self.assertEqual(config['init']*4, value)

        counter = Counter()
        while not head_queue.queue.empty():
            msg = head_queue.queue.get(1)
            counter[msg.joint_name] += 1

        # every motor has gotten a command message
        self.assertEqual((set(head_pau_motors)-set(counter.keys())), set([]))

    def test_lips_pau_message(self):
        reader = PololuSerialReader('%s/pololu1' % CWD)
        self.runner.config.nodes = filter(
            lambda node: node.name != 'blender_api', self.runner.config.nodes)
        self.runner.launch()
        time.sleep(2) # wait for pau2motors

        lips_pau_motors = set(rosparam.get_param('/sophia/pau2motors/topics')['lips_pau'].split(';'))
        dynamixel_motors = {'Frown_L', 'Frown_R', 'Smile_L', 'Smile_R', 'Jaw_Up_Down'}
        lips_pau_motors = lips_pau_motors - dynamixel_motors
        motors = rosparam.get_param('/sophia/motors')
        pololu_motor_config = {}
        for motor in motors:
            if 'motor_id' in motor:
                pololu_motor_config[motor['motor_id']] = motor

        self.assertTrue(len(lips_pau_motors)>0)

        lips_pau_messages = MessageQueue()
        lips_pau_messages.subscribe('/sophia/lips_pau', pau)

        lips_queue = MessageQueue()
        lips_queue.subscribe('/sophia/head/command', MotorCommand)
        time.sleep(1)

        pau_message = pau()
        pub, msg_class = rostopic.create_publisher('/sophia/lips_pau', 'pau2motors/pau', True)
        pub.publish(msg_class(m_coeffs=[0]*48))
        time.sleep(2)

        print 'lips pau %s' % lips_pau_messages.queue.qsize()
        for _ in range(len(lips_pau_motors)*3):
            motor_id, cmd, value = reader.read()
            config = pololu_motor_config[motor_id]
            if cmd == 'position':
                print motor_id, cmd, value
                self.assertEqual(config['init']*4, value)

        counter = Counter()
        while not lips_queue.queue.empty():
            msg = lips_queue.queue.get(1)
            counter[msg.joint_name] += 1

        # every motor has gotten a command message
        self.assertEqual((lips_pau_motors-set(counter.keys())), set([]))

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_sophia2', SophiaTest2)
