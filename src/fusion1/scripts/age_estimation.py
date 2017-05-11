#!/usr/bin/env python2.7
import os
import rospy
import logging
import time
import cv2
import subprocess
import csv

from sensor_msgs.msg import Image;
from fusion1.msg import AgeRequest,AgeResponse;
from cv_bridge import CvBridge;

from std_msgs.msg import String;


opencv_bridge = CvBridge()


class AgeEstimation(object):


    def __init__(self):

        # start subscribers and publishers
        self.request_sub = rospy.Subscriber("age_request",AgeRequest,self.HandleAgeRequest)
        self.response_pub = rospy.Publisher("age_response",AgeResponse,queue_size=5)

        # start timer to indicate age estimation is ready for a next request
        self.timer = rospy.Timer(rospy.Duration(5.0),self.HandleTimer)


    def HandleAgeRequest(self,data):

        # various filenames
        br_name = "/opt/hansonrobotics/vision/openbr/build/app/br/br"
        cur_name = os.getenv("HOME") + "/hansonrobotics/HEAD/src/fusion1/test"
        image_name = cur_name + "/age.bmp"
        csv_name = cur_name + "/age.csv"

        # save image
        img = opencv_bridge.imgmsg_to_cv2(data.image,"8UC1")
        cv2.imwrite(image_name,img)

        # run commandline age estimation
        subprocess.call([br_name,"-useGui","0","-algorithm","AgeEstimation","-enroll",image_name,csv_name])

        # interpret the resulting CSV file
        results = []
        with open(csv_name,"rb") as file:
            reader = csv.reader(file)
            reader.next()
            results = reader.next()

        # and publish the result
        response = AgeResponse()
        response.id = data.id
        response.ts = data.ts
        response.age = float(results[1])
        response.confidence = 1.0 # TODO: this tho
        self.response_pub.publish(response)


    def HandleTimer(self,event):

        # publish empty result
        response = AgeResponse()
        response.id = 0
        response.ts = rospy.Time(0,0)
        response.age = 0.0
        response.confidence = 0.0
        self.response_pub.publish(response)


if __name__ == '__main__':

    # initialize node
    rospy.init_node('age_estimation')

    # create age estimation object
    node = AgeEstimation()

    # run
    rospy.spin()
