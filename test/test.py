#!/usr/bin/env python

import unittest
import time
from functools import partial

import rospy
from ros_pololu.msg import *
from std_msgs.msg import Float64
from motors_safety import Safety

def motor_type(motor):
    if 'motor_id' in motor:
        return 'pololu'
    else:
        return 'dynamixel'

class MotorSafetyTest(unittest.TestCase):
    # Test data for the motor rules
    _TEST_MOTORS = [
        {
            'name': 'motor1',
            'topic': 'motor1',
            'motor_id': 3,
            'default': 0.1,
            'min': -0.5,
            'max': 1.5,
        },
        {
            'name': 'motor2',
            'topic': 'motor2',
            'default': 0,
            'min': -0.5,
            'max': 1.2,
        },
    ]
    _TEST_RULES = {
        'motor1': [
            {
                'type': 'timing',
                'direction': 'min',
                'extreme': -0.4,
                't1': 1,
                't2': 1,
                't3': 5,
                't4': 1,
            }
        ]
    }
    _TIMEOUT  = 0.1

    def __init__(self, *args, **kwargs):
        super(MotorSafetyTest, self).__init__(*args, **kwargs)
        rospy.init_node('motors_safety_test')
        # Data for actual motors
        rospy.set_param('motors', self._TEST_MOTORS)
        rospy.set_param('safety_rules', self._TEST_RULES)

    def setUp(self):
        self.motors = self._TEST_MOTORS
        self.safety = Safety()
        # Message was passed
        self.proxy_pass = False
        # Wrong value
        self.safe_val = 100
        # In case there are issues
        self.ignore_msgs = False
        # Create publishers and subscribers for testing
        self.pub = {}
        self.sub = {}
        for m in self.motors:
            if motor_type(m) == 'pololu':
                topic = '%s/command' % m['topic']
                self.pub[m['topic']] = rospy.Publisher(topic, MotorCommand, queue_size=30)
                safe_topic = 'safe/%s/command' % m['topic']
                self.sub[m['topic']] = rospy.Subscriber(safe_topic, MotorCommand, partial(self.check_safe_msgs, motor=m))
            else:
                topic = '%s_controller/command' % m['topic']
                self.pub[m['topic']] = rospy.Publisher(topic, Float64, queue_size=30)
                safe_topic = 'safe/%s_controller/command' % m['topic']
                self.sub[m['topic']] = rospy.Subscriber(safe_topic, Float64, partial(self.check_safe_msgs, motor=m))
        time.sleep(self._TIMEOUT)

        pass
    # Checks the recieved messages for given values
    def check_safe_msgs(self, msg, motor):
        if self.ignore_msgs:
            return
        if motor_type(motor) == 'pololu':
            self.safe_val = msg.position
        else:
            self.safe_val = msg.data
        self.proxy_pass = True


    # Creates message with value
    def create_msg(self, motor, v):
        if motor_type(motor) == 'pololu':
            msg = MotorCommand()
            msg.position = v
            msg.joint_name = motor['name']
        else:
            msg = Float64()
            msg.data = v
        return msg

    def assertMessageVal(self, expected, motor):
        self.assertTrue(self.proxy_pass, 'Message was not forwarded for %s' % motor['name'])
        self.assertAlmostEqual(expected, self.safe_val, msg="Wrong safe value. Expected %1.3f got %1.3f for %s" %
                                                            (expected,self.safe_val,motor['name']))
    # Messages should pass without changes to the safe topic for neutral values.
    def send_default_messages(self):
        for m in self.motors:
            self.proxy_pass = False
            msg = self.create_msg(m, m['default'])
            self.pub[m['topic']].publish(msg)
            time.sleep(self._TIMEOUT)
            self.assertMessageVal(m['default'], m)

    #Simple test for default messages
    def test_default_messages(self):
        self.send_default_messages()



    def test_set_motor_relative_position(self):
        # Sends initial default messages
        self.send_default_messages()

        for m in self.motors:
            self.proxy_pass = False
            self.safety.set_motor_relative_pos(m['name'], 0.9, 'max')
            time.sleep(self._TIMEOUT)
            self.assertMessageVal(m['default'] + (m['max']-m['default'])*0.9, m)

if __name__ == "__main__":
    unittest.main()