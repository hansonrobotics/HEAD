import unittest
import time
from functools import partial

import rospy
from ros_pololu.msg import *
from std_msgs.msg import Float64

def motor_type(motor):
    if 'motor_id' in motor:
        return 'pololu'
    else:
        return 'dynamixel'

class MotorSafetyTest(unittest.TestCase):
    def setUp(self):
        rospy.init_node('motors_safety_test')
        self.motors = rospy.get_param('motors')

    def check(self, msg, msg_orig, motor, timestamp, timeout=10):
        delta = time.time() - timestamp
        self.assertLess(delta, timeout, "Timeout")
        if motor_type(motor) == 'pololu':
            self.assertEqual(motor['topic'], msg_orig['joint_name'])
            self.assertEqual(motor['topic'], msg['joint_name'])

    def test(self):
        for m in self.motors:
            if motor_type(m) == 'pololu':
                topic = '%s/command' % m['topic']
                safe_topic = 'safe/%s/command' % m['topic']
                pub = rospy.Publisher(topic, MotorCommand, queue_size=30)
                msg_orig = MotorCommand(
                    joint_name=m['topic'], position=m['max'],
                    speed=0.5, acceleration=0.5)
            else:
                topic = '%s_controller/command' % m['topic']
                safe_topic = 'safe/%s_controller/command' % m['topic']
                pub = rospy.Publisher(topic, Float64, queue_size=30)
                msg_orig = Float64(m['max'])

            sub = rospy.Subscriber(
                safe_topic, MotorCommand,
                partial(self.check, msg_orig=msg_orig,
                        motor=m, timestamp=time.time()))
            pub.publish(msg_orig)
            time.sleep(1)

