import unittest
import time
import os
from functools import partial

import rospy
from ros_pololu.msg import *
from std_msgs.msg import Float64

def motor_type(motor):
    if 'motor_id' in motor:
        return 'pololu'
    else:
        return 'dynamixel'

MSG_TYPES = {'pololu': MotorCommand, 'dynamixel': Float64}
MOTOR_COMMAND_TOPICS = {
    'pololu': '%s/command',
    'dynamixel': '%s_controller/command'
    }

class Timeout(Exception):
    pass

class MotorSafetyTest(unittest.TestCase):
    def setUp(self):
        os.system('rosparam load safety_rules.yaml test')
        rospy.init_node('motors_safety_test')
        self.ns = '/eva'
        motors = rospy.get_param('%s/motors' % self.ns)
        self.rules = rospy.get_param('test/safety_rules')
        self.motors = {m['name']: m for m in motors}
        self.timeout = 5
        self.got_msg = False

    def check(self, msg, msg_orig, motor, timestamp, rule):
        """
        pololu msg:
            string joint_name
            float64 position
            float32 speed
            float32 acceleration
        """
        delta = time.time() - timestamp
        self.assertLess(delta, self.timeout, "Timeout")
        extreme = rule['extreme']
        if motor_type(motor) == 'pololu':
            self.assertEqual(motor['topic'], msg_orig['joint_name'])
            self.assertEqual(motor['topic'], msg['joint_name'])
            self.assertLessEqual(msg.position/motor['max'], extreme)
        else:
            self.assertLessEqual(msg.data/motor['max'], extreme)
        self.got_msg = True

    def send_motor_cmd(self, motor, position):
        topic = MOTOR_COMMAND_TOPICS[motor_type(motor)] % motor['topic']
        topic = '%s/%s' % (self.ns, topic)
        msg_type = MSG_TYPES[motor_type(motor)]
        pub = rospy.Publisher(topic, msg_type, queue_size=30)
        if motor_type(motor) == 'pololu':
            msg = MotorCommand(
                joint_name=motor['topic'], position=position,
                speed=0.5, acceleration=0.5)
        else:
            msg = Float64(position)
        pub.publish(msg)
        return topic, msg

    def get_command_safe_topic(self, motor):
        return 'safe/%s' % self.get_command_topic(motor)

    def get_msg_type(self, motor):
        if motor_type(motor) == 'pololu':
            msg_type = MotorCommand
        else:
            msg_type = Float64
        return msg_type

    def test(self):
        for name in self.rules:
            motor = self.motors[name]
            rules = self.rules[name]
            # Send two related motor commands at the same time and
            # expect the position in safe/*/command topic to send to motor
            # is limited.
            for rule in rules:
                dep_motor = self.motors[rule['depends']]
                self.send_motor_cmd(dep_motor, dep_motor['max'])
                topic, msg = self.send_motor_cmd(motor, motor['max'])

                safe_topic = '%s/safe/%s' % (
                    self.ns,
                    MOTOR_COMMAND_TOPICS[motor_type(motor)] % motor['topic'])

                sub = rospy.Subscriber(
                    safe_topic, MSG_TYPES[motor_type(motor)],
                    partial(self.check, msg_orig=msg,
                            motor=motor, timestamp=time.time(), rule=rule))

                while not self.got_msg:
                    time.sleep(self.timeout)

                if not self.got_msg:
                    raise Timeout
                self.got_msg = False

    def tearDown(self):
        os.system('rosparam delete test')

if __name__ == '__main__':
    unittest.main()

