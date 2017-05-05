#!/usr/bin/env python2.7
import os
import rospy
import logging

from fusion1.msg import ...
# dynamic reconfigure server?

logger = logging.getLogger('hr.fusion1')


class DetectGaze(object):

    def __init__(self):
    	self.face_landmarks_sub = rospy.Subscriber("face_landmarks",FaceLandmarks,self.HandleFaceLandmarks)
    	self.gaze_pub = rospy.Publisher("gaze",Gaze,queue_size=1)
    	self.timer = rospy.Timer(...,self.HandleTimer)

    def HandleFaceLandmarks(self,data):
    	# replace current face landmarks

    def HandleTimer(self):
		# if no current face landmarks, return
		# execute gaze detection algorithm on current face landmarks
		# publish gaze data


if __name__ == '__main__':
    rospy.init_node('detect_gaze')
    node = DetectGaze()
    rospy.spin()
