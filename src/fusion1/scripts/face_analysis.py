#!/usr/bin/env python2.7
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
        print "Request to analyze face: session %08X, camera %08X, puser %08X, face %08X" % (data.session_id & 0xFFFFFFFF,data.camera_id & 0xFFFFFFFF,data.puser_id & 0xFFFFFFFF,data.face_id & 0xFFFFFFFF)

        response = FaceResponse()
        response.session_id = data.session_id
        response.camera_id = data.camera_id
        response.puser_id = data.puser_id
        response.face_id = data.face_id
        response.ts = data.ts
        response.age = 30
        response.age_confidence = 0.0
        response.gender = 1
        response.gender_confidence = 0.0
        response.identity = 0
        response.identity_confidence = 0.0
        self.response_pub.publish(response)


if __name__ == '__main__':

    rospy.init_node('face_analysis')
    node = FaceAnalysis()
    rospy.spin()
