#!/usr/bin/env python2.7

# R2 Perception - Hanson Robotics Unified Perception System, v1.0
# by Desmond Germans

# HAND DETECTION: template

# HAND DETECTION: analyze camera frame and output all found hands at a configurable rate
#     the camera is a ROS USB camera node under 'camera' in the local namespace
#     the raw hands are published to 'raw_hand' in the local namespace
#     parameter updates are gathered from the 'vision_pipeline' parameter server

# the node should be called 'detect_hands'

import os
import rospy
import logging
import time
import cv2
import dynamic_reconfigure.client
from sensor_msgs.msg import Image
from fusion1.msg import Hand
from cv_bridge import CvBridge


# create OpenCV-ROS bridge object
opencv_bridge = CvBridge()


# Generate unique serial number for the raw hands 
serial_number = 0
def GenerateHandID():
    global serial_number
    result = serial_number
    serial_number += 1
    return result


class DetectHands(object):


    # constructor
    def __init__(self):

        # initialize current frame and timestamp
        self.cur_image = Image()
        self.cur_ts = 0.0

        # get dynamic reconfigure parameters
        self.fovy = rospy.get_param("fovy")
        self.aspect = rospy.get_param("aspect")
        self.hand_detect_rate = rospy.get_param("hand_detect_rate")
        self.dynparam = dynamic_reconfigure.client.Client("vision_pipeline",timeout=30,config_callback=self.HandleConfig)        

        # start subscriber and publisher
        self.image_sub = rospy.Subscriber("camera/image_raw",Image,self.HandleImage)
        self.hand_pub = rospy.Publisher("raw_hand",Hand,queue_size=5)

        # start timer
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.hand_detect_rate),self.HandleTimer)


    # when a dynamic reconfigure update occurs
    def HandleConfig(self,data):

        # copy parameters from server
        self.fovy = data.fovy
        self.aspect = data.aspect
        self.hand_detect_rate = data.hand_detect_rate

        # set new timer period
        self.timer.setPeriod(rospy.Duration(1.0 / self.hand_detect_rate))


    # when a camera image arrives
    def HandleImage(self,data):

        self.cur_image = data
        self.cur_ts = rospy.get_rostime()


    # at face detection rate
    def HandleTimer(self,event):

        # if no image is available, exit
        if self.cur_ts == 0.0:
            return

        # TEMPLATE: detect all hands

        # TEMPLATE: if there are no hands detected, exit

        # TEMPLATE: publish all hands


if __name__ == '__main__':

    rospy.init_node('detect_hands')
    node = DetectHands()
    rospy.spin()
