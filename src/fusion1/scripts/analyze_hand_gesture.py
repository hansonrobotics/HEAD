#!/usr/bin/env python2.7
import os
import rospy
import logging

from fusion1.msg import ...
# dynamic reconfigure server?

logger = logging.getLogger('hr.fusion1')


class AnalyzeHandGesture(object):

    def __init__(self):
    	self.hand_landmarks_sub = rospy.Subscriber("hand_landmarks",HandLandmarks,self.HandleHandLandmarks)
    	self.hand_gesture_pub = rospy.Publisher("hand_gesture",HandGesture,queue_size=1)
    	self.timer = rospy.Timer(...,self.HandleTimer)

    def HandleHandLandmarks(self,data):
    	# replace current hand landmarks

    def HandleTimer(self):
		# if no current hand landmarks, return
		# execute gesture detection algorithm on current hand landmarks
		# publish hand gesture data


if __name__ == '__main__':
    rospy.init_node('analyze_hand_gesture')
    node = AnalyzeHandGesture()
    rospy.spin()
