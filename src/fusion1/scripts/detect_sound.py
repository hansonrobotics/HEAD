#!/usr/bin/env python2.7

# R2 Perception - Hanson Robotics Unified Perception System, v1.0
# by Desmond Germans

import os
import rospy
import logging
import time
import dynamic_reconfigure.client
from fusion1.msg import Sound


# speech to text
class DetectSound(object):


    def __init__(self):

        self.threshold = rospy.get_param("threshold")
        self.linger = rospy.get_param("linger")
        self.dynparam = dynamic_reconfigure.client.Client("hearing_pipeline",timeout=30,config_callback=self.HandleConfig)        

        self.sound_pub = rospy.Publisher("sound",Sound,queue_size=5)


    def HandleConfig(self,data):

        self.threshold = data.threshold
        self.linger = data.linger


if __name__ == '__main__':

    rospy.init_node('detect_sound')
    node = DetectSound()
    rospy.spin()
