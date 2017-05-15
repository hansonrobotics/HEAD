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
from fusion1.msg import Face,Hand,FaceRequest,FaceResponse;
from partial_user import PartialUser;

# base directory to store face thumbs
thumb_base_dir = os.getenv("HOME") + "/hansonrobotics/HEAD/src/fusion1/test/thumbs"
thumb_ext = ".bmp"

# user ID serial number
serial_number = -1

# maximum distance for face match
face_continuity_threshold_m = 0.5

# minimum confidence needed to reliably say this is a face
minimum_confidence = 0.4

# invalidation time difference
time_difference = rospy.Time(1,0)

# prepare OpenCV-ROS bridge
opencv_bridge = CvBridge()

# prepare font


def GeneratePartialUserID():

    global serial_number
    serial_number += 1
    return serial_number


class VisionPipeline(object):


    def __init__(self,name,invocation_id):

        # store administrative info
        self.name = name
        self.invocation_id = invocation_id

        # create pipeline ID from the name
        self.pipeline_id = hash(name) & 0xFFFFFFFF

        # create thumb output directories
        today_tag = time.strftime("%Y%m%d")
        invocation_tag = "invocation%08X" % (self.invocation_id & 0xFFFFFFFF)
        pipeline_tag = "pipeline%08X" % (self.pipeline_id & 0xFFFFFFFF)
        self.thumb_dir = thumb_base_dir + "/" + today_tag + "/" + invocation_tag + "/" + pipeline_tag + "/"
        if not os.path.exists(self.thumb_dir):
            os.makedirs(self.thumb_dir)

        # initialize partial users
        self.pusers = {}

        # initialize face request queue
        self.face_request_queue = []

        # start subscribers and publishers
        self.face_sub = rospy.Subscriber(self.name + "/face",Face,self.HandleFace)
        self.hand_sub = rospy.Subscriber(self.name + "/hand",Hand,self.HandleHand)
        self.face_response_sub = rospy.Subscriber(self.name + "/face_response",FaceResponse,self.HandleFaceResponse)
        self.face_request_pub = rospy.Publisher(self.name + "/face_request",FaceRequest,queue_size=5)

        # for debugging
        cv2.startWindowThread()
        cv2.namedWindow(self.name)
        self.frame_sub = rospy.Subscriber(self.name + "/camera/image_raw",Image,self.HandleFrame)


    def HandleFace(self,data):

        global face_continuity_threshold_m

        closest_puser_id = 0
        closest_dist = face_continuity_threshold_m

        # find partial user that would end up closest to this face
        for puser_id in self.pusers:
            face = self.pusers[puser_id].Extrapolate(data.ts)
            dx = data.pos.x - face.pos.x
            dy = data.pos.y - face.pos.y
            dz = data.pos.z - face.pos.z
            d = sqrt(dx * dx + dy * dy + dz * dz)
            if (closest_puser_id == 0) or (d < closest_dist):
                closest_puser_id = puser_id
                closest_dist = d

        if closest_dist < face_continuity_threshold_m:

            # append this face to the partial user
            self.pusers[closest_puser_id].Append(data)

        else:

            # create new partial user
            closest_puser_id = GeneratePartialUserID()
            puser = PartialUser()
            puser.Append(data)

            # add to partial users
            self.pusers[closest_puser_id] = puser

            # enqueue request for face analysis
            request = FaceRequest()
            request.invocation_id = self.invocation_id
            request.pipeline_id = self.pipeline_id
            request.puser_id = id
            request.face_id = data.id
            request.ts = data.ts
            request.thumb = data.thumb
            face_request_queue.append(request)

        # save thumb in directory structure (indexed by date, pipeline and partial user ID)
        puser_tag = "puser%08X" % (closest_puser_id & 0xFFFFFFFF)
        dir = self.thumb_dir + puser_tag + "/"
        if not os.path.exists(dir):
            os.makedirs(dir)
        thumb = opencv_bridge.imgmsg_to_cv2(data.thumb,"8UC3")
        face_tag = "face%08X" % (data.face_id & 0xFFFFFFFF)
        thumb_file = dir + face_tag + thumb_ext
        cv2.imwrite(thumb_file,thumb)


    def HandleHand(self,data):

        () # do stuff with users


    def HandleFaceResponse(self,data):

        # TODO: pop from the queue, and publish next request if any

        if data.invocation_id == 0: # the face analysis node outputs empty responses to push a new message from this queue
            return

        # if partial user doesn't exist anymore, forget about this
        if data.puser_id not in self.pusers:
            return

        # store analyzed data
        self.pusers[data.puser_id].age = data.age
        self.pusers[data.puser_id].age_confidence = data.age_confidence
        self.pusers[data.puser_id].gender = data.gender
        self.pusers[data.puser_id].gender_confidence = data.gender_confidence
        self.pusers[data.puser_id].identity = data.identity
        self.pusers[data.puser_id].identity_confidence = data.identity_confidence


    # for debugging
    def HandleFrame(self,data):

        # get image
        frame = opencv_bridge.imgmsg_to_cv2(data,"bgr8")

        # get time
        ts = rospy.get_rostime()

        # display users
        for puser_id in self.pusers:

            # the face
            face = self.pusers[puser_id].Extrapolate(ts)
            x = int((0.5 + 0.5 * face.pos.x) * 640.0)
            y = int((0.5 + 0.5 * face.pos.y) * 480.0)
            cv2.circle(frame,(x,y),10,(0,255,255),2)

            # annotate with info if available
            label = ""
            if self.pusers[puser_id].identity_confidence > 0.0:
                label = self.pusers[puser_id].identity
            if self.pusers[puser_id].age_confidence > 0.0:
                if label != "":
                    label += ", "
                label += str(int(self.pusers[puser_id].age)) + " y/o"
            if self.pusers[puser_id].gender_confidence > 0.0:
                if label != "":
                    label += ", "
                if gender == 0:
                    label += "genderless"
                elif gender == 1:
                    label += "female"
                elif gender == 2:
                    label += "male"
            cv2.putText(frame,label,(x - 20,y + 20),cv2.cv.CV_FONT_HERSHEY_PLAIN,1,(0,255,255))

        cv2.imshow(self.name,frame)


    def Step(self,ts):

        global minimum_confidence
        global time_difference

        # mine the current partial users
        #result_pusers = []
        #for puser_id in self.pusers:
        #    conf = self.pusers[puser_id].CalculateConfidence()
        #    if conf >= minimum_confidence:
        #        result_pusers.append(self.pusers[puser_id].Extrapolate(ts))

        # prune the partial users and remove them if they disappeared
        to_be_removed = []
        prune_before_time = ts - time_difference
        for puser_id in self.pusers:
            self.pusers[puser_id].PruneBefore(prune_before_time)
            if len(self.pusers[puser_id].faces) == 0:
                print "puser list is empty, remove it"
                to_be_removed.append(puser_id)
        for key in to_be_removed:
            print "removing puser %08X" % (puser_id & 0xFFFFFFFF)
            del self.pusers[puser_id]
