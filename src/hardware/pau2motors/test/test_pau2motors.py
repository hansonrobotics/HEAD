#!/usr/bin/env python

import unittest
import os
import subprocess
import atexit
import rospy
import math

from pau2motors.msg import pau
from std_msgs.msg import Float64
from testing_tools.misc import wait_for, MessageQueue

CWD = os.path.abspath(os.path.dirname(__file__))

cmd = 'roslaunch {}/test_pau2motors.test'.format(CWD)
proc = subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid)
def shutdown():
    if proc:
        os.killpg(proc.pid, 2)
atexit.register(shutdown)

class Pau2MotorsTest(unittest.TestCase):

    @classmethod
    def setUpClass(self):
        rospy.init_node('pau2motors_test')
        wait_for('pau2motors')
        os.system('rosrun dynamic_reconfigure dynparam set /pau2motors reload True')
        self.queue = MessageQueue()
        self.pub = rospy.Publisher('/listen_topic', pau, queue_size=1)

    def check_msg(self, msg, expect):
        self.queue.clear()
        self.pub.publish(msg)
        msg = self.queue.get(2)
        self.assertEqual(msg.data, expect)

    def test_motor1(self):
        msg = pau()
        msg.m_headRotation.x = 0.5
        msg.m_headRotation.y = 0.5
        msg.m_headRotation.z = 0.0
        msg.m_headRotation.w = 1.0
        sub = self.queue.subscribe('/pub_topic_controller/command', Float64)
        rospy.sleep(2)
        self.check_msg(msg, math.asin(0.5))
        sub.unregister()

    def test_motor2(self):
        msg = pau()
        msg.m_neckRotation.x = 1.0
        msg.m_neckRotation.y = 0.5
        msg.m_neckRotation.z = 0.0
        msg.m_neckRotation.w = 1.0
        sub = self.queue.subscribe('/pub_topic2_controller/command', Float64)
        rospy.sleep(2)
        self.check_msg(msg, 0.5) # max = 0.5
        sub.unregister()

if __name__ == '__main__':
    unittest.main()

