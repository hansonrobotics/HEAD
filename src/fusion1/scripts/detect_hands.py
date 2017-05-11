#!/usr/bin/env python2.7
import os
import rospy
import logging
import time
import cv2
from sensor_msgs.msg import Image;
from fusion1.msg import Hand;
from cv_bridge import CvBridge;
from std_msgs.msg import String;


# prepare OpenCV-ROS bridge and TBD hand detection stuff
opencv_bridge = CvBridge()


# prepare serial ID, unique to this namespace
ros_namespace_hash = 0
serial_number = 0

def GenerateID():
    global ros_namespace_hash
    global serial_number
    result = ros_namespace_hash + serial_number
    serial_number += 1
    return result


# the hand detector
class DetectHands(object):


    def __init__(self):

        # initialize current image and timestamp
        self.most_recent_frame = Image()
        self.most_recent_ts = 0.0

        # initialize face detection period (can be changed later by ROS service message)
        self.period_sec = 0.2 # 5Hz

        # start subscribers and publishers
        self.frame_sub = rospy.Subscriber("camera/image_raw",Image,self.HandleFrame)
        self.hand_pub = rospy.Publisher("hand",Hand,queue_size=5)

        # start timer
        self.timer = rospy.Timer(rospy.Duration(self.period_sec),self.HandleTimer)


    def HandleFrame(self,data):

        # overwrite current frame and generate timestamp
        self.most_recent_frame = data
        self.most_recent_ts = rospy.get_rostime()


    def HandleTimer(self,event):

        # make sure there is a frame to process
        if self.most_recent_ts == 0.0:
            return

        # TODO: detect hands

        # TODO: don't publish anything if there were no hands detected

        # TODO: publish all hands


if __name__ == '__main__':

    # initialize node
    rospy.init_node('detect_hands')

    # initialize namespace hash
    ros_namespace_hash = hash(rospy.get_namespace())

    # create face detector object
    node = DetectHands()

    # run
    rospy.spin()
