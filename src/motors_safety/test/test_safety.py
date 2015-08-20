#!/usr/bin/env python

import unittest
import time
from functools import partial
from mock import *

import rospy
from ros_pololu.msg import *
from std_msgs.msg import Float64
from motors_safety import Safety
from dynamixel_msgs.msg import *

def motor_type(motor):
    if 'motor_id' in motor:
        return 'pololu'
    else:
        return 'dynamixel'

mock_time = Mock()
mock_time.return_value = time.time()

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
            'min': -0.6,
            'max': 1.2,
        },
    ]
    _TEST_RULES = {
        'motor1': [
            {
                'type': 'timing',
                'direction': 'min',
                'extreme': 0.8,
                't1': 1,
                't2': 1,
                't3': 5,
                't4': 1,
            }
        ],
        'motor2': [
            {
                'type': 'prevent',
                'direction': 'max',
                'extreme': 0.5,
                'depends': 'motor1',
                'dep_dir': 'max',
                'dep_extreme': 0.6
            },
            {
                'type': 'load',
                'direction': 'min',
                'extreme': 0.4,
                'motor_id': 2,
                'rest': 0.1,
                't1': 1,
                't2': 4,
            }
        ]
    }
    _TIMEOUT = 0.1

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
        self.motor_states_pub = rospy.Publisher("safe/motor_states", MotorStateList, queue_size=10)

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

    def assertMessageVal(self, expected, motor, equal=True):
        self.assertTrue(self.proxy_pass, 'Message was not forwarded for %s' % motor['name'])
        if equal:
            self.assertAlmostEqual(expected, self.safe_val, msg="Wrong safe value. Expected %1.3f got %1.3f for %s" %
                                                            (expected,self.safe_val,motor['name']))
        else:
            self.assertNotAlmostEqual(expected, self.safe_val,msg="Expected not equal values, got equal values %1.3f for %s" %
                                                            (expected, motor['name']))
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

    # Tests the rule for timing
    @patch('time.time', mock_time)
    def test_rule_timing(self):
        # Init the default messages
        self.send_default_messages()
        start = time.time()
        # Send extreme position
        m = self.motors[0]
        self.proxy_pass = False
        msg = self.create_msg(m, self.safety.get_abs_pos('motor1','min',0.9,))
        self.pub['motor1'].publish(msg)
        time.sleep(self._TIMEOUT)
        self.assertMessageVal(m['default'] + (m['min']-m['default'])*0.9, m)
        self.safety.timing()
        # Extreme value should be not limited in 1.2s
        mock_time.return_value = start + 1.2
        msg = self.create_msg(m, self.safety.get_abs_pos('motor1','min',0.9))
        self.proxy_pass = False
        self.pub['motor1'].publish(msg)
        self.safety.timing()
        time.sleep(self._TIMEOUT)
        self.assertMessageVal(m['default'] + (m['min']-m['default'])*0.9, m)
        # Extreme position should be on its way down in 2nd second
        mock_time.return_value = start + 1.80
        self.safety.timing()
        self.proxy_pass = False
        msg = self.create_msg(m, self.safety.get_abs_pos('motor1','min',0.9))
        self.pub['motor1'].publish(msg)
        time.sleep(self._TIMEOUT)
        self.assertMessageVal(m['default'] + (m['min']-m['default'])*(0.8+0.2*0.2), m)
        # Extreme position should be limited for t3 period
        mock_time.return_value = start + 2.1
        self.safety.timing()
        self.proxy_pass = False
        msg = self.create_msg(m, self.safety.get_abs_pos('motor1','min',0.9))
        self.pub['motor1'].publish(msg)
        time.sleep(self._TIMEOUT)
        self.assertMessageVal(m['default'] + (m['min']-m['default'])*0.8, m)
        mock_time.return_value = start + 7
        self.safety.timing()
        self.proxy_pass = False
        msg = self.create_msg(m, self.safety.get_abs_pos('motor1','min',0.9))
        self.pub['motor1'].publish(msg)
        time.sleep(self._TIMEOUT)
        self.assertMessageVal(m['default'] + (m['min']-m['default'])*0.8, m)
        # Extreme position should be back in t4
        mock_time.return_value = start + 7.5
        self.safety.timing()
        self.proxy_pass = False
        msg = self.create_msg(m, self.safety.get_abs_pos('motor1','min',0.9))
        self.pub['motor1'].publish(msg)
        time.sleep(self._TIMEOUT)
        self.assertMessageVal(m['default'] + (m['min']-m['default'])*0.9, m)

    # Test prevention rule
    def test_prevent(self):
        # Send the default messages
        self.send_default_messages()
        # Sends message close to extreme that would limit motor2
        msg = self.create_msg(self.motors[0], self.safety.get_abs_pos('motor1','max',0.55))
        self.pub['motor1'].publish(msg)
        time.sleep(self._TIMEOUT)
        # Sends the extreme message to motor2
        self.proxy_pass = False
        m = self.motors[1]
        msg = self.create_msg(m, self.safety.get_abs_pos('motor2','max',0.95))
        self.pub['motor2'].publish(msg)
        time.sleep(self._TIMEOUT)
        # Expects the value to not be limited
        self.assertMessageVal(m['default'] + (m['max']-m['default'])*0.95, m)
        # Sending the first motor value that would limit motor 2
        msg = self.create_msg(self.motors[0], self.safety.get_abs_pos('motor1','max',0.65))
        self.pub['motor1'].publish(msg)
        time.sleep(self._TIMEOUT)
         # Sends the extreme message to motor2
        self.proxy_pass = False
        m = self.motors[1]
        msg = self.create_msg(m, self.safety.get_abs_pos('motor2','max',0.95))
        self.pub['motor2'].publish(msg)
        time.sleep(self._TIMEOUT)
        # Expects the value to not be limited to its limit before extreme position
        self.assertMessageVal(m['default'] + (m['max']-m['default'])*0.5, m)


    def send_motor_state(self, id, load):
        state = MotorState()
        state.id = id
        state.load = load
        states = MotorStateList()
        states.motor_states = [state]
        self.motor_states_pub.publish(states)

    # Test the laod related changes. Applies only for dynamixels
    @patch('time.time', mock_time)
    def test_load(self):
        # Send the default messages
        # Send load not reaching threshold
        m = self.motors[1]
        self.send_motor_state(2, -0.35)
        time.sleep(self._TIMEOUT)
        self.safety.timing()
        self.assertFalse(self.safety.rules['motor2'][1]['started'],msg= "Unexpected rule start")
        # Send extreme message
        msg = self.create_msg(m, self.safety.get_abs_pos('motor2','min',0.95))
        self.pub['motor2'].publish(msg)
        time.sleep(self._TIMEOUT)
        #Start
        start = time.time()
        self.send_motor_state(2, -0.45)
        time.sleep(self._TIMEOUT)
        self.safety.timing()
        self.assertEqual(self.safety.rules['motor2'][1]['started'], start, msg="Wrong starting time")
        # Check value before starting to decline
        mock_time.return_value = start + 0.99
        self.safety.timing()
        self.proxy_pass = False
        msg = self.create_msg(m, self.safety.get_abs_pos('motor2','min',0.95))
        self.pub['motor2'].publish(msg)
        time.sleep(self._TIMEOUT)
        self.assertMessageVal(m['default'] + (m['min']-m['default'])*0.95, m)

        # Limit should start declining
        mock_time.return_value = start + 1.1
        self.safety.timing()
        msg = self.create_msg(m, self.safety.get_abs_pos('motor2','min',0.95))
        self.pub['motor2'].publish(msg)
        time.sleep(self._TIMEOUT)
        #Expects the value to be changed by filter
        self.assertMessageVal(m['default'] + (m['min']-m['default'])*0.95, m, equal=False)
        # Each timing call should decrease the motor limit towards natural
        self.safety.timing()
        # previous value
        expected = self.safe_val
        self.proxy_pass = False
        msg = self.create_msg(m, self.safety.get_abs_pos('motor2','min',0.95))
        self.pub['motor2'].publish(msg)
        time.sleep(self._TIMEOUT)
        self.assertMessageVal(expected, m, equal=False)

        # Send the motor to recovered state
        self.send_motor_state(2,0.18)
        time.sleep(self._TIMEOUT)
        self.safety.timing()

        # The expected value should be same as the last value after motor is recovered
        expected = self.safe_val
        self.assertNotEqual(expected, self.safety.get_abs_pos('motor2','min',0.95), msg="Expected value should be different")
        self.proxy_pass = False
        msg = self.create_msg(m, self.safety.get_abs_pos('motor2','min',0.95))
        self.pub['motor2'].publish(msg)
        time.sleep(self._TIMEOUT)
        # value should keep consistent
        self.assertMessageVal(expected, m)
        # Keeps motor until rule expires
        mock_time.return_value = start + 4.9
        self.safety.timing()
        self.proxy_pass = False
        msg = self.create_msg(m, self.safety.get_abs_pos('motor2','min',0.95))
        self.pub['motor2'].publish(msg)
        time.sleep(self._TIMEOUT)
        # Check if same limit applies
        self.assertMessageVal(expected,m)
        # After time is passed limit is gradually increased with timing call
        mock_time.return_value = start + 5.1
        self.safety.timing()
        msg = self.create_msg(m, self.safety.get_abs_pos('motor2','min',0.95))
        self.pub['motor2'].publish(msg)
        time.sleep(self._TIMEOUT)
        self.assertMessageVal(expected, m, equal=False)
        # Limit is beack to extreme position after multiple timing calls
        # Should be similar number of calls as for decreasing the limit
        self.safety.timing()
        self.safety.timing()
        self.safety.timing()
        # Check if extreme messages are not modified after returniong back to normal value
        self.proxy_pass = False
        msg = self.create_msg(m, self.safety.get_abs_pos('motor2','min',0.99))
        self.pub['motor2'].publish(msg)
        time.sleep(self._TIMEOUT)
        self.assertMessageVal(m['default'] + (m['min']-m['default'])*0.99,m)


if __name__ == "__main__":
    import rostest
    rostest.rosrun('motors_safety', 'test', MotorSafetyTest)