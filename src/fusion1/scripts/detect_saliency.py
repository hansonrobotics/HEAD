#!/usr/bin/env python2.7
import os
import rospy
import logging

from fusion1.msg import ...
# dynamic reconfigure server?

logger = logging.getLogger('hr.fusion1')


class DetectSaliency(object):

    def __init__(self):
    	self.image_sub = rospy.Subscriber("camera_feed",Image,self.HandleImage)
    	self.saliency_pub = rospy.Publisher("saliency",Saliency,queue_size=1)
    	self.timer = rospy.Timer(...,self.HandleTimer)

    def HandleImage(self,data):
    	# get timestamp
    	# replace current timestamped image

	def HandleTimer(self):
		# if no current timestamped image, return
		# generate new uid
		# execute saliency detection algorithm on current timestamped image
		# publish timestamped saliency event


if __name__ == '__main__':
    rospy.init_node('detect_saliency')
    node = DetectSaliency()
    rospy.spin()
