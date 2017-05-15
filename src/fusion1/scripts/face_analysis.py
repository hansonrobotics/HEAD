#!/usr/bin/env python2.7
import os
import rospy
import logging
import time
import cv2
import subprocess
import csv

from sensor_msgs.msg import Image;
from fusion1.msg import FaceRequest,FaceResponse;
from cv_bridge import CvBridge;

from std_msgs.msg import String;


opencv_bridge = CvBridge()


class FaceAnalysis(object):


    def __init__(self):

        # start subscribers and publishers
        self.request_sub = rospy.Subscriber("face_request",FaceRequest,self.HandleFaceRequest)
        self.response_pub = rospy.Publisher("face_response",FaceResponse,queue_size=5)


    def HandleFaceRequest(self,data):

        # TODO: run shitloads of fuckloads of TensorFlow magic on data.thumb
        print "Request to analyze face: invocation %08X, pipeline %08X, puser %08X, face %08X" % (data.invocation_id & 0xFFFFFFFF,data.pipeline_id & 0xFFFFFFFF,data.puser_id & 0xFFFFFFFF,data.face_id & 0xFFFFFFFF)

        # various filenames
        #br_name = os.getenv("HOME") + "/openbr/build/app/br/br"
        #cur_name = os.getenv("HOME") + "/hansonrobotics/HEAD/src/fusion1/test"
        #image_name = cur_name + "/age.bmp"
        #csv_name = cur_name + "/age.csv"

        # save image
        #img = opencv_bridge.imgmsg_to_cv2(data.image,"8UC1")
        #cv2.imwrite(image_name,img)

        # run commandline age estimation
        #subprocess.call([br_name,"-useGui","0","-algorithm","AgeEstimation","-enroll",image_name,csv_name])

        # interpret the resulting CSV file
        #results = []
        #with open(csv_name,"rb") as file:
        #    reader = csv.reader(file)
        #    reader.next()
        #    results = reader.next()

        # and publish the result
        response = FaceResponse()
        response.invocation_id = data.invocation_id
        response.pipeline_id = data.pipeline_id
        response.puser_id = data.puser_id
        response.face_id = data.face_id
        response.ts = data.ts
        response.age = 30
        response.age_confidence = 0.0
        response.gender = 1
        response.gender_confidence = 0.0
        response.identity = "unknown"
        response.identity_confidence = 0.0
        self.response_pub.publish(response)


if __name__ == '__main__':

    # initialize node
    rospy.init_node('face_estimation')

    # create age estimation object
    node = FaceAnalysis()

    # run
    rospy.spin()
