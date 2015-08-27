#!/usr/bin/env python

import unittest
import rospy
import time

class MotorSafetyTest(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(MotorSafetyTest, self).__init__(*args, **kwargs)
        rospy.init_node('motors_safety_test')

    def test_eye_tracking(self):
        time.sleep(5)
        self.assertTrue(True)


if __name__ == "__main__":
    import rostest
    rostest.rosrun('motors_safety', 'test', MotorSafetyTest)