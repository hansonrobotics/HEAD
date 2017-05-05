#!/usr/bin/env python2.7
import os
import rospy
import numpy
import logging
import time
import cv2

from sensor_msgs.msg import Image;
from fusion1.msg import floatxy,Face,Thumb;
from cv_bridge import CvBridge;
from math import sqrt;

logger = logging.getLogger('hr.fusion1')


# maximum distance for face match
face_continuity_threshold = 0.1

# number of trail points for 100% confidence
full_trail_n = 5

# minimum confidence needed to reliably say this is a face
minimum_confidence = 0.4

# trail invalidation time difference
time_difference = rospy.Time(1,0)


class Point(object):


    def __init__(self,ts=0,uid=0,x=0.0,y=0.0):

        self.ts = ts
        self.uid = uid
        self.x = x
        self.y = y


class Trail(object):


    def __init__(self):

        self.points = []


    def Extrapolate(self,ts):

        n = len(self.points)
        if n < 2:
            return self.points[0]
        sumx = 0.0
        sumxx = 0.0
        sumy = 0.0
        sumyy = 0.0
        sumt = 0.0
        sumtt = 0.0
        sumtx = 0.0
        sumty = 0.0
        for point in self.points:
            x = point.x
            y = point.y
            t = (point.ts - ts).to_sec()
            sumx += x
            sumxx += x * x
            sumy += y
            sumyy += y * y
            sumt += t
            sumtt += t * t
            sumtx += t * x
            sumty += t * y
        xslp = (n * sumtx - sumt * sumx) / (n * sumtt - sumt * sumt)
        rx = (sumx - xslp * sumt) / n
        yslp = (n * sumty - sumt * sumy) / (n * sumtt - sumt * sumt)
        ry = (sumy - yslp * sumt) / n
        return floatxy(rx,ry)


    def PruneBefore(self,ts):

        oldest = 0
        for i in range(0,len(self.points)):
            if self.points[i].ts.to_sec() < ts.to_sec():
                oldest = i + 1 # because Python...
        del self.points[:oldest]


    def Append(self,point):

        self.points.append(point)


    def CalculateConfidence(self):

        global full_trail_n

        n = len(self.points)
        if n > full_trail_n:
            n = full_trail_n
        return float(n) / float(full_trail_n)


class Fusion(object):


    def __init__(self):

        # for debugging
        cv2.startWindowThread()
        cv2.namedWindow("vision")
        self.allimage = numpy.zeros((560,480,3),numpy.uint8)
        self.opencv_bridge = CvBridge()

        # initialize current face trails
        self.wideangle_trails = []
        self.lefteye_trails = []
        self.righteye_trails = []
        self.realsense_trails = []

        # initialize user output period
        self.period_sec = 0.03333 # 30Hz

        # start subscribers and publishers
        self.wideangle_face_sub = rospy.Subscriber("wideangle/face",Face,self.HandleWideAngleFace)
        self.wideangle_thumb_sub = rospy.Subscriber("wideangle/thumb",Thumb,self.HandleWideAngleThumb)
        self.lefteye_face_sub = rospy.Subscriber("lefteye/face",Face,self.HandleLeftEyeFace)
        self.lefteye_thumb_sub = rospy.Subscriber("lefteye/thumb",Thumb,self.HandleLeftEyeThumb)
        self.righteye_face_sub = rospy.Subscriber("righteye/face",Face,self.HandleRightEyeFace)
        self.righteye_thumb_sub = rospy.Subscriber("righteye/thumb",Thumb,self.HandleRightEyeThumb)
        self.timer = rospy.Timer(rospy.Duration(self.period_sec),self.HandleTimer)

        # for debugging
        self.wideangle_sub = rospy.Subscriber("wideangle/camera/image_raw",Image,self.HandleWideAngleImage)
        self.lefteye_sub = rospy.Subscriber("lefteye/camera/image_raw",Image,self.HandleLeftEyeImage)
        self.righteye_sub = rospy.Subscriber("righteye/camera/image_raw",Image,self.HandleRightEyeImage)
        #self.realsense_sub = rospy.Subscriber("realsense/camera/image_raw",Image,self.HandleRealSenseImage)


    def HandleWideAngleFace(self,data):

        # whenever a face comes in from the wide-angle camera face detection,
        # figure out to which trail it belongs to and append it or create a new trail

        global face_continuity_threshold

        closest_i = -1
        closest_dist = face_continuity_threshold

        # find trail that would end up closest to the face
        for i in range(0,len(self.wideangle_trails)):
            p = self.wideangle_trails[i].Extrapolate(data.ts)
            dx = data.rect.center.x - p.x
            dy = data.rect.center.y - p.y
            d = sqrt(dx * dx + dy * dy)
            if d < closest_dist:
                closest_i = i
                closest_dist = d

        point = Point(data.ts,data.uid,data.rect.center.x,data.rect.center.y)

        # if found, append this face to the trail
        if closest_dist < face_continuity_threshold:
            self.wideangle_trails[closest_i].Append(point)

        # otherwise, add new trail ("a new face appears")
        else:
            trail = Trail()
            trail.Append(point)
            self.wideangle_trails.append(trail)


    def HandleWideAngleThumb(self,data):

        ()
        #cvimage = self.opencv_bridge.imgmsg_to_cv2(data.image,"8UC1")
        #cvbigimage = cv2.resize(cvimage,(64,64))
        #self.allimage[0:64,0:64] = cv2.cvtColor(cvbigimage,cv2.COLOR_GRAY2BGR)


    def HandleLeftEyeFace(self,data):

        global face_continuity_threshold

        closest_i = -1
        closest_dist = face_continuity_threshold

        # find trail that would end up closest to the face
        for i in range(0,len(self.lefteye_trails)):
            p = self.lefteye_trails[i].Extrapolate(data.ts)
            dx = data.rect.center.x - p.x
            dy = data.rect.center.y - p.y
            d = sqrt(dx * dx + dy * dy)
            if d < closest_dist:
                closest_i = i
                closest_dist = d

        point = Point(data.ts,data.uid,data.rect.center.x,data.rect.center.y)

        # if found, append this face to the trail
        if closest_dist < face_continuity_threshold:
            self.lefteye_trails[closest_i].Append(point)

        # otherwise, add new trail ("a new face appears")
        else:
            trail = Trail()
            trail.Append(point)
            self.lefteye_trails.append(trail)


    def HandleLeftEyeThumb(self,data):

        ()
        #cvimage = self.opencv_bridge.imgmsg_to_cv2(data.image,"8UC1")
        #cvbigimage = cv2.resize(cvimage,(64,64))
        #self.allimage[64:128,0:64] = cv2.cvtColor(cvbigimage,cv2.COLOR_GRAY2BGR)


    def HandleRightEyeFace(self,data):

        ()


    def HandleRightEyeThumb(self,data):

        ()


    #def HandleRealSenseFace(self,data):

    #    ()


    def HandleWideAngleImage(self,data):

        cvimage = self.opencv_bridge.imgmsg_to_cv2(data,"bgr8")
        self.allimage[320:560,80:400] = cv2.resize(cvimage,(320,240))


    def HandleLeftEyeImage(self,data):

        cvimage = self.opencv_bridge.imgmsg_to_cv2(data,"bgr8")
        matrix = cv2.getRotationMatrix2D((240,320),-90,0.5)
        self.allimage[0:320,0:240] = cv2.warpAffine(cvimage,matrix,(240,320))


    def HandleRightEyeImage(self,data):

        cvimage = self.opencv_bridge.imgmsg_to_cv2(data,"bgr8")
        matrix = cv2.getRotationMatrix2D((240,320),-90,0.5)
        self.allimage[0:320,240:480] = cv2.warpAffine(cvimage,matrix,(240,320))


    def HandleTimer(self,data):

        # at regular intervals (or when a camera image appears in debugging setup),
        # render and prune the trails, maintaining a notion of users with confidence
        # this exploits spatial and temporal continuity in the incoming data to remove
        # false positives and false negatives

        global minimum_confidence
        global time_difference

        ts = data.current_expected


        # WIDEANGLE FACES:

        # mine the trails for currently existing users
        result_wideangle_faces = []
        for i in range(0,len(self.wideangle_trails)):
            conf = self.wideangle_trails[i].CalculateConfidence()
            if conf >= minimum_confidence:
                result_wideangle_faces.append(self.wideangle_trails[i].Extrapolate(ts))

        # prune the trails
        to_be_removed = []
        prune_before_time = ts - time_difference
        for i in range(0,len(self.wideangle_trails)):
            self.wideangle_trails[i].PruneBefore(prune_before_time)
            if len(self.wideangle_trails[i].points) == 0:
                to_be_removed.append(i)
        for i in reversed(to_be_removed):
            del self.wideangle_trails[i]


        # LEFT EYE FACES:

        # mine the trails for currently existing users
        result_lefteye_faces = []
        for i in range(0,len(self.lefteye_trails)):
            conf = self.lefteye_trails[i].CalculateConfidence()
            if conf >= minimum_confidence:
                result_lefteye_faces.append(self.lefteye_trails[i].Extrapolate(ts))

        # prune the trails
        to_be_removed = []
        prune_before_time = ts - time_difference
        for i in range(0,len(self.lefteye_trails)):
            self.lefteye_trails[i].PruneBefore(prune_before_time)
            if len(self.lefteye_trails[i].points) == 0:
                to_be_removed.append(i)
        for i in reversed(to_be_removed):
            del self.lefteye_trails[i]


        # RIGHT EYE FACES:


        # REALSENSE FACES:


        # for debugging
        for point in result_wideangle_faces:
            x = 160 + int((0.5 + 0.5 * point.x) * 320.0)
            y = 0 + int((0.5 + 0.5 * point.y) * 240.0)
            cv2.rectangle(self.allimage, (x - 5,y - 5), (x + 5,y + 5), (0,0,255), 1)

        for point in result_lefteye_faces:
            x = 0 + int((0.5 + 0.5 * point.x) * 320.0)
            y = 240 + int((0.5 + 0.5 * point.y) * 240.0)
            cv2.rectangle(self.allimage, (x - 5,y - 5), (x + 5,y + 5), (0,0,255), 1)

        cv2.imshow("vision",self.allimage)



if __name__ == '__main__':
    rospy.init_node('fusion')
    node = Fusion()
    rospy.spin()
