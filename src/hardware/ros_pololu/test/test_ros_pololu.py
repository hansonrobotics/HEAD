#!/usr/bin/env python

import unittest
import os
import time
import subprocess
import serial

import rostopic
import roslaunch
from roslaunch import core
from roslaunch import nodeprocess
nodeprocess._TIMEOUT_SIGINT = 2
nodeprocess._TIMEOUT_SIGTERM = 1

from testing_tools.misc import PololuSerialReader

CWD = os.path.abspath(os.path.dirname(__file__))
PKG = 'ros_pololu'

class TestROSPololu(unittest.TestCase):

    def setUp(self):
        self.run_id = 'test_ros_pololu'

        # ros_pololu_node
        launch_file = '%s/robot.launch' % CWD
        config = roslaunch.config.load_config_default([launch_file], None)

        self.socat_procs = []
        # virtual serial port
        serial_ports = {
            '/han/pololu_left/port_name': ('left_pololu0', 'left_pololu1'),
            '/han/pololu_middle/port_name': ('middle_pololu0', 'middle_pololu1'),
            '/han/pololu_right/port_name': ('right_pololu0', 'right_pololu1'),
            '/han/dynamixel_manager/serial_ports/pan_tilt_port/port_name': ('ttyUSB0', 'ttyUSB1')
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

        self.runner.launch()

    def tearDown(self):
        self.runner.stop()
        for proc in self.socat_procs:
            os.killpg(proc.pid, 2)

    def test_right_pololu_message(self):
        reader = PololuSerialReader('%s/right_pololu1' % CWD)
        topic = '/han/right/command'
        pub, msg_class = rostopic.create_publisher(
            topic, 'ros_pololu/MotorCommand', True)

        time.sleep(3) # wait for pololu to be connected

        loop = 100
        for i in range(loop):
            pub.publish(msg_class('EyeLid_L_R', 0.1, 2.0, 2.0)) # 2
            pub.publish(msg_class('Eyelid_U_R', 0.1, 2.0, 2.0)) # 1

        for i in range(3*2*loop):
            id, cmd, value = reader.read()
            print "set motor %s %s value %s" % (id, cmd, value)

    def test_left_pololu_message(self):
        reader = PololuSerialReader('%s/left_pololu1' % CWD)
        topic = '/han/left/command'
        pub, msg_class = rostopic.create_publisher(
            topic, 'ros_pololu/MotorCommand', True)

        time.sleep(3) # wait for pololu to be connected

        loop = 100
        for i in range(loop):
            pub.publish(msg_class('EyeLid_L_L', 0.1, 2.0, 2.0))
            pub.publish(msg_class('EyeLid_U_L', 0.1, 2.0, 2.0))

        for i in range(3*2*loop):
            id, cmd, value = reader.read()
            print "set motor %s %s value %s" % (id, cmd, value)

    def test_middle_pololu_message(self):
        reader = PololuSerialReader('%s/middle_pololu1' % CWD)
        topic = '/han/middle/command'
        pub, msg_class = rostopic.create_publisher(
            topic, 'ros_pololu/MotorCommand', True)

        time.sleep(3) # wait for pololu to be connected

        loop = 100
        for i in range(loop):
            pub.publish(msg_class('Brow_Inner_L', 0.1, 2.0, 2.0)) #1
            pub.publish(msg_class('Inner_Brow_R_UD', 0.1, 2.0, 2.0)) #4

        for i in range(3*2*loop):
            id, cmd, value = reader.read()
            print "set motor %s %s value %s" % (id, cmd, value)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_ros_polou', TestROSPololu)

