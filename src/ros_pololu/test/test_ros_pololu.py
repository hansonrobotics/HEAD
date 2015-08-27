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


CWD = os.path.abspath(os.path.dirname(__file__))
PKG = 'ros_pololu'
CMD_DICT = {'135': 'speed', '137': 'accelaration', '132': 'position'}

class SerialReader(object):
    def __init__(self, device):
        self.ser = serial.Serial(device, baudrate=115200,
                                bytesize=serial.EIGHTBITS,
                                parity=serial.PARITY_NONE,
                                stopbits=serial.STOPBITS_ONE,
                                timeout=5, writeTimeout=None)

    def read(self):
        try:
            num = self.ser.read(size=4)
            id = ord(num[1])
            cmd = CMD_DICT[str(ord(num[0]))]
            value = ord(num[3])*128+ord(num[2])
        except serial.SerialException as e:
            raise e
        except TypeError as e:
            id, cmd, value = 0, '', 0
        return (id, cmd, value)

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
            '/han/safe/dynamixel_manager/serial_ports/pan_tilt_port/port_name': ('ttyUSB0', 'ttyUSB1')
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

    def test_ros_message(self):
        reader = SerialReader('%s/right_pololu1' % CWD)
        topic = '/han/safe/right/command'
        pub, msg_class = rostopic.create_publisher(
            topic, 'ros_pololu/MotorCommand', True)

        pub.publish(msg_class('EyeLid_L_R', 0.1, 2.0, 2.0))
        time.sleep(1)
        pub.publish(msg_class('Eyelid_U_R', 0.1, 2.0, 2.0))
        for i in range(3):
            id, cmd, value = reader.read()
            print "set motor %s %s value %s" % (id, cmd, value)

        for i in range(3):
            id, cmd, value = reader.read()
            print "set motor %s %s value %s" % (id, cmd, value)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_ros_polou', TestROSPololu)

