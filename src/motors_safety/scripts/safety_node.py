#!/usr/bin/env python3

import rospy
from motors_safety import Safety
import time

ROS_RATE = 20

if __name__ == '__main__':
    # Wait for motors to be loaded in param server
    time.sleep(3)
    rospy.init_node('motors_safety')
    MS = Safety()
    MS.ros_rate = ROS_RATE
    r = rospy.Rate(ROS_RATE)
    while not rospy.is_shutdown():
        MS.timing()
        r.sleep()