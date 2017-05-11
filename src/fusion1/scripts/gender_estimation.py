#!/usr/bin/env python2.7
import os
import rospy
import logging
import time
import cv2
import subprocess
import csv

from sensor_msgs.msg import Image;
from fusion1.msg import GenderRequest,GenderResponse;
from cv_bridge import CvBridge;

from std_msgs.msg import String;


opencv_bridge = CvBridge()


class GenderEstimation(object):


    def __init__(self):

        # start subscribers and publishers
        self.request_sub = rospy.Subscriber("gender_request",GenderRequest,self.HandleGenderRequest)
        self.response_pub = rospy.Publisher("gender_response",GenderResponse,queue_size=5)

        # start timer to indicate age estimation is ready for a next request
        self.timer = rospy.Timer(rospy.Duration(5.0),self.HandleTimer)


    def HandleGenderRequest(self,data):

        # various filenames
        br_name = "/opt/hansonrobotics/vision/openbr/build/app/br/br"
        cur_name = os.getenv("HOME") + "/hansonrobotics/HEAD/src/fusion1/test"
        image_name = cur_name + "/gender.bmp"
        csv_name = cur_name + "/gender.csv"

        # save image
        img = opencv_bridge.imgmsg_to_cv2(data.image,"8UC1")
        cv2.imwrite(image_name,img)

        # run commandline age estimation
        subprocess.call([br_name,"-useGui","0","-algorithm","GenderEstimation","-enroll",image_name,csv_name])

        # interpret the resulting CSV file
        results = []
        with open(csv_name,"rb") as file:
            reader = csv.reader(file)
            reader.next() # skip headers
            results = reader.next()

        # and publish the result
        response = GenderResponse()
        response.id = data.id
        response.ts = data.ts
        if results[10] == "Female":
            response.female = True
        else:
            response.female = False
        response.confidence = 1.0 # TODO: this tho
        self.response_pub.publish(response)


    def HandleTimer(self,event):

        # publish empty result
        response = GenderResponse()
        response.id = 0
        response.ts = rospy.Time(0,0)
        response.gender = False
        response.confidence = 0.0
        self.response_pub.publish(response)


if __name__ == '__main__':

    # initialize node
    rospy.init_node('gender_estimation')

    # create gender estimation object
    node = GenderEstimation()

    # run
    rospy.spin()
