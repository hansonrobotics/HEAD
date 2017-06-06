#!/usr/bin/env python2.7
import os
import rospy
import logging
import time
import dynamic_reconfigure.client
from fusion1.msg import Sound


# speech to text
class DetectSound(object):


    def __init__(self):

        rospy.wait_for_service("hearing_pipeline")
        self.dynparam = dynamic_reconfigure.client.Client("hearing_pipeline",timeout=30,config_callback=self.HandleConfig)        
        self.threshold = rospy.get_param("threshold")
        self.linger = rospy.get_param("linger")

        self.sound_pub = rospy.Publisher("sound",Sound,queue_size=5)


    def HandleConfig(self,data):

        print "detect_sound {}".format(data)


if __name__ == '__main__':

    rospy.init_node('detect_sound')
    node = DetectSound()
    rospy.spin()
