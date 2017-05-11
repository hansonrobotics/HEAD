#!/usr/bin/env python2.7
import os
import rospy
import numpy
import logging
import time
import cv2

from sensor_msgs.msg import Image;
from cv_bridge import CvBridge;
from math import sqrt;
from fusion1.msg import Face,Hand,AgeRequest,AgeResponse,GenderRequest,GenderResponse;
from partial_user import PartialUser,Point;


# maximum distance for face match
face_continuity_threshold_m = 0.5

# minimum confidence needed to reliably say this is a face
minimum_confidence = 0.4

# invalidation time difference
time_difference = rospy.Time(1,0)

# prepare OpenCV-ROS bridge
opencv_bridge = CvBridge()

# prepare font


# prepare serial ID, unique to this namespace
ros_namespace_hash = 0
serial_number = 0

def GenerateID():

    global ros_namespace_hash
    global serial_number
    result = ros_namespace_hash + serial_number
    serial_number += 1
    return result


class VisionPipeline(object):


    def __init__(self,name):

        self.name = name

        # initialize partial users
        self.users = {}

        # start subscribers and publishers
        self.face_sub = rospy.Subscriber(self.name + "/face",Face,self.HandleFace)
        self.hand_sub = rospy.Subscriber(self.name + "/hand",Hand,self.HandleHand)
        self.age_response_sub = rospy.Subscriber(self.name + "/age_response",AgeResponse,self.HandleAgeResponse)
        self.age_request_pub = rospy.Publisher(self.name + "/age_request",AgeRequest,queue_size=5)
        self.gender_response_sub = rospy.Subscriber(self.name + "/gender_response",GenderResponse,self.HandleGenderResponse)
        self.gender_request_pub = rospy.Publisher(self.name + "/gender_request",GenderRequest,queue_size=5)

        # for debugging
        cv2.startWindowThread()
        cv2.namedWindow(self.name)
        self.frame_sub = rospy.Subscriber(self.name + "/camera/image_raw",Image,self.HandleFrame)


    def HandleFace(self,data):

        global face_continuity_threshold_m

        closest_key = None
        closest_dist = face_continuity_threshold_m

        # find partial user that would end up closest to this face
        for key in self.users:
            p = self.users[key].Extrapolate(data.ts)
            dx = data.x - p.x
            dy = data.y - p.y
            dz = data.z - p.z
            d = sqrt(dx * dx + dy * dy + dz * dz)
            if (closest_key == None) or (d < closest_dist):
                closest_key = key
                closest_dist = d

        point = Point(data.ts,data.x,data.y,data.z)

        if closest_dist < face_continuity_threshold_m:

            # append this face to the partial user
            self.users[closest_key].Append(point)

        else:

            # create new partial user
            id = GenerateID()
            user = PartialUser()
            user.Append(point)

            # add to partial users
            self.users[id] = user

            # request for age estimation
            # TODO: this should probably be a queue, replacing requests for the same partial user
            request = AgeRequest()
            request.id = id
            request.ts = data.ts
            request.image = data.image
            self.age_request_pub.publish(request)

            # request for gender estimation
            # TODO: this should probably be a queue, replacing requests for the same partial user
            request = GenderRequest()
            request.id = id
            request.ts = data.ts
            request.image = data.image
            self.gender_request_pub.publish(request)


    def HandleHand(self,data):

        () # do stuff with users


    def HandleAgeResponse(self,data):

        # TODO: pop from the queue
        if data.id == 0:
            return

        # if partial user still exists, update age and confidence
        self.users[data.id].age = data.age
        self.users[data.id].age_confidence = data.confidence

        print "age = {}".format(self.users[data.id].age)


    def HandleGenderResponse(self,data):

        # TODO: pop from the queue
        if data.id == 0:
            return

        # if partial user still exists, update age and confidence
        self.users[data.id].female = data.female
        self.users[data.id].female_confidence = data.confidence

        if self.users[data.id].female:
            print "female"
        else:
            print "male"


    # for debugging
    def HandleFrame(self,data):

        # get image
        frame = opencv_bridge.imgmsg_to_cv2(data,"bgr8")

        # get time
        ts = rospy.get_rostime()

        # display users
        for key in self.users:

            # the face
            point = self.users[key].Extrapolate(ts)
            x = int((0.5 + 0.5 * point.x) * 640.0)
            y = int((0.5 + 0.5 * point.y) * 480.0)
            cv2.circle(frame,(x,y),10,(0,255,255),2)

            # annotate with age and gender
            label = ""
            if self.users[key].female_confidence > 0.0:
                if self.users[key].female:
                    label = "female"
                else:
                    label = "male"
            if self.users[key].age_confidence > 0.0:
                age = str(int(self.users[key].age))
                if label != "":
                    label += ", "
                label += age + " y/o"
            cv2.putText(frame,label,(x - 20,y + 20),cv2.cv.CV_FONT_HERSHEY_PLAIN,1,(0,255,255))

        cv2.imshow(self.name,frame)


    def Step(self,ts):

        global minimum_confidence
        global time_difference

        # mine the current partial users
        result_users = []
        for key in self.users.keys():
            conf = self.users[key].CalculateConfidence()
            if conf >= minimum_confidence:
                result_users.append(self.users[key].Extrapolate(ts))

        # prune the partial users
        to_be_removed = []
        prune_before_time = ts - time_difference
        for key in self.users.keys():
            self.users[key].PruneBefore(prune_before_time)
            if len(self.users[key].points) == 0:
                to_be_removed.append(key)
        for key in to_be_removed:
            del self.users[key]
