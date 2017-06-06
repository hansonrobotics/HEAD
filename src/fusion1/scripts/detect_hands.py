#!/usr/bin/env python2.7
import os
import rospy
import logging
import time
import cv2
import dynamic_reconfigure.client
from sensor_msgs.msg import Image
from fusion1.msg import Hand
from cv_bridge import CvBridge


opencv_bridge = CvBridge()


serial_number = 0
def GenerateHandID():
    global serial_number
    result = serial_number
    serial_number += 1
    return result


class DetectHands(object):


    def __init__(self):

        self.cur_image = Image()
        self.cur_ts = 0.0

        self.fovy = rospy.get_param("fovy")
        self.aspect = rospy.get_param("aspect")
        self.hand_detect_rate = rospy.get_param("hand_detect_rate")
        rospy.wait_for_service("vision_pipeline")
        self.dynparam = dynamic_reconfigure.client.Client("vision_pipeline",timeout=30,config_callback=self.HandleConfig)        

        self.image_sub = rospy.Subscriber("camera/image_raw",Image,self.HandleImage)
        self.hand_pub = rospy.Publisher("raw_hand",Hand,queue_size=5)

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.hand_detect_rate),self.HandleTimer)


    def HandleConfig(self,data):

        print "detect_hands {}".format(data)


    def HandleImage(self,data):

        self.cur_image = data
        self.cur_ts = rospy.get_rostime()


    def HandleTimer(self,event):

        if self.cur_ts == 0.0:
            return

        # TODO: detect hands

        # TODO: don't publish anything if there were no hands detected

        # TODO: publish all hands


if __name__ == '__main__':

    rospy.init_node('detect_hands')
    node = DetectHands()
    rospy.spin()
