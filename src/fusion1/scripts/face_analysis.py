#!/usr/bin/env python2.7

# R2 Perception - Hanson Robotics Unified Perception System, v1.0
# by Desmond Germans

import os
import rospy
import time
import cv2

from sensor_msgs.msg import Image
from fusion1.msg import FaceRequest,FaceResponse
from cv_bridge import CvBridge


opencv_bridge = CvBridge()


class FaceAnalysis(object):


    def __init__(self):

        self.request_sub = rospy.Subscriber("face_request",FaceRequest,self.HandleFaceRequest)
        self.response_pub = rospy.Publisher("face_response",FaceResponse,queue_size=5)


    def HandleFaceRequest(self,data):

        # TODO: run shitloads of fuckloads of TensorFlow magic on data.thumb
        print "Request to analyze face: session %08X, camera %08X, cuser %08X, face %08X" % (data.session_id & 0xFFFFFFFF,data.camera_id & 0xFFFFFFFF,data.cuser_id & 0xFFFFFFFF,data.face_id & 0xFFFFFFFF)

        msg = FaceResponse()
        msg.session_id = data.session_id
        msg.camera_id = data.camera_id
        msg.cuser_id = data.cuser_id
        msg.face_id = data.face_id
        msg.ts = data.ts
        msg.age = 30
        msg.age_confidence = 0.0
        msg.gender = 1
        msg.gender_confidence = 0.0
        msg.identity = 0
        msg.identity_confidence = 0.0
        self.response_pub.publish(msg)


if __name__ == '__main__':

    rospy.init_node('face_analysis')
    node = FaceAnalysis()
    rospy.spin()
