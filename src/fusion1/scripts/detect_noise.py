#!/usr/bin/env python2.7
import os
import rospy
import logging

from fusion1.msg import ...
# dynamic reconfigure server?

logger = logging.getLogger('hr.fusion1')


class DetectNoise(object):

    def __init__(self):
    	self.audio_sub = rospy.Subscriber("audio",Audio,self.HandleAudio)
    	self.noise_pub = rospy.Publisher("noise",Noise,queue_size=1)

    def HandleAudio(self,data):
    	# analyze samples, detect if there is loud noise, if so output noise event
    	# ...take cooldown period


if __name__ == '__main__':
    rospy.init_node('detect_noise')
    node = DetectNoise()
    rospy.spin()
