#!/usr/bin/env python2.7
import os
import rospy
import logging

from fusion1.msg import ...
# dynamic reconfigure server?

logger = logging.getLogger('hr.fusion1')


class DetectHand(object):

    def __init__(self):
    	self.image_sub = rospy.Subscriber("camera_feed",Image,self.HandleImage)
    	self.hand_pub = rospy.Publisher("hand",HandRect,queue_size=1)
    	self.timer = rospy.Timer(...,self.HandleTimer)

    def HandleImage(self,data):
    	# get timestamp
    	# replace current timestamped image

	def HandleTimer(self):
		# if no current timestamped image, return
		# generate new uid
		# execute hand detection algorithm on current timestamped image
		# publish timestamped hand rectangle


if __name__ == '__main__':
    rospy.init_node('detect_hand')
    node = DetectHand()
    rospy.spin()
