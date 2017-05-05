#!/usr/bin/env python2.7
import os
import rospy
import logging

from fusion1.msg import ...
# dynamic reconfigure server?

logger = logging.getLogger('hr.fusion1')


class AnalyzeFaceExpression(object):

    def __init__(self):
    	self.face_landmarks_sub = rospy.Subscriber("face_landmarks",FaceLandmarks,self.HandleFaceLandmarks)
    	self.face_expression_pub = rospy.Publisher("face_expression",FaceExpression,queue_size=1)
    	self.timer = rospy.Timer(...,self.HandleTimer)

    def HandleFaceLandmarks(self,data):
    	# replace current face landmarks

    def HandleTimer(self):
		# if no current face landmarks, return
		# execute face expression detection algorithm on current face landmarks
		# publish face expression data


if __name__ == '__main__':
    rospy.init_node('analyze_face_expression')
    node = AnalyzeFaceExpression()
    rospy.spin()
