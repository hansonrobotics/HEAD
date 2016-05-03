#!/usr/bin/env python

import unittest
import rospy
import time
from pau2motors.msg import pau

class EyeTrackingTest(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(EyeTrackingTest, self).__init__(*args, **kwargs)
        rospy.init_node('eyes_tracking_test')

    def setUp(self):
        self.sub = rospy.Subscriber("eyes_pau", pau, self.last_message)
        self.get_pau = rospy.Publisher("/blender_api/get_pau", pau, queue_size=10)
        self.last_msg = False
        time.sleep(1)
        msg = pau()
        msg.m_eyeGazeLeftYaw = 1
        self.get_pau.publish(msg)
        time.sleep(1)
        rospy.logwarn("Setup")


    def last_message(self, msg):
        self.last_msg = msg

    def test_message_fwd(self):
        #IF node active the pau messages should be forwarded from /blender_api to eyes_pau
        msg = pau()
        self.last_msg = False
        self.get_pau.publish(msg)
        time.sleep(0.1)
        self.assertTrue(self.last_msg != False, "Message not received")
        # Check if eye positions been modified
        t = time.time()
        modified = False
        #Checks if eyes positions has been changed
        while time.time() - t < 3:
            self.get_pau.publish(msg)
            time.sleep(1)
            if self.last_msg.m_eyeGazeRightPitch != msg.m_eyeGazeRightPitch:
                modified = True
                break
        self.assertTrue(modified, "Eye positions were not adjusted")











if __name__ == "__main__":
    import rostest
    rostest.rosrun('eyes_tracking', 'test', EyeTrackingTest)