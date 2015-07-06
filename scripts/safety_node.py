#!/usr/bin/env python

import rospy
from motors_safety import Safety
from ros_pololu.msg import MotorCommand
from std_msgs.msg import Float64
from dynamixel_msgs.msg import MotorStateList

import time

ROS_RATE = 20

if __name__ == '__main__':
    rospy.init_node('motors_safety')
    MS = Safety()
    MS.ros_rate = ROS_RATE
    r = rospy.Rate(ROS_RATE)
    while not rospy.is_shutdown():
        MS.timing()
        r.sleep()