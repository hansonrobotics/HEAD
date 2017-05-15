#!/usr/bin/env python2.7
import os
import rospy
import logging
import time
import cv2
from sensor_msgs.msg import Image;
from fusion1.msg import Face;
from cv_bridge import CvBridge;
from std_msgs.msg import String;


# prepare OpenCV-ROS bridge and face Haar cascade
opencv_bridge = CvBridge()
face_cascade = cv2.CascadeClassifier(os.getenv("HOME") + "/hansonrobotics/HEAD/src/fusion1/test/haarcascade_frontalface_alt.xml")
if face_cascade.empty():
    print "cascade not found"


# prepare calculation of 3D face

fixed_face_width_m = 0.17
camera_fovy_atan = 1.0 # TODO: specify real camera fovy


# prepare serial ID, unique to this namespace
serial_number = -1

def GenerateFaceID():
    global serial_number
    serial_number += 1
    return serial_number


# the face detector
class DetectFaces(object):


    def __init__(self):

        # initialize current image and timestamp
        self.most_recent_frame = Image()
        self.most_recent_ts = 0.0

        # initialize face detection period (can be changed later by ROS service message)
        self.period_sec = 0.2 # 5Hz

        # start subscribers and publishers
        self.frame_sub = rospy.Subscriber("camera/image_raw",Image,self.HandleFrame)
        self.face_pub = rospy.Publisher("face",Face,queue_size=5)

        # start timer
        self.timer = rospy.Timer(rospy.Duration(self.period_sec),self.HandleTimer)


    def HandleFrame(self,data):

        # overwrite current frame and generate timestamp
        self.most_recent_frame = data
        self.most_recent_ts = rospy.get_rostime()


    def HandleTimer(self,event):

        global fixed_face_width
        global camera_fovy_atan

        # make sure there is a frame to process
        if self.most_recent_ts == 0.0:
            return

        # detect faces
        color_frame = opencv_bridge.imgmsg_to_cv2(self.most_recent_frame,"bgr8")
        faces = face_cascade.detectMultiScale(color_frame,scaleFactor=1.1,minSize=(30,30),flags=cv2.cv.CV_HAAR_SCALE_IMAGE)

        # don't publish anything if there were no faces detected
        if len(faces) == 0:
            return

        # publish all faces
        for (x,y,w,h) in faces:

            if (w <= 0) or (h <= 0):
                continue

            # prepare face message
            face = Face()

            # create new UID for the face
            face.face_id = GenerateFaceID()

            # set timestamp
            face.ts = self.most_recent_ts

            # set rectangle
            face.rect.origin.x = -1.0 + 2.0 * float(x) / float(self.most_recent_frame.width)
            face.rect.origin.y = -1.0 + 2.0 * float(y) / float(self.most_recent_frame.height)
            face.rect.size.x = 2.0 * float(w) / float(self.most_recent_frame.width)
            face.rect.size.y = 2.0 * float(h) / float(self.most_recent_frame.height)

            # TODO: convert rectangle to 3D coordinates
            face.pos.x = face.rect.origin.x + face.rect.size.x / 2
            face.pos.y = face.rect.origin.y + face.rect.size.y / 2
            face.pos.z = 1.0

            # assume it's a valid face
            face.confidence = 1.0

            # non-realsense does not understand smile, frown and expressions
            face.smile = 0.0
            face.frown = 0.0
            face.expressions = []
            
            # landmarks
            face.landmarks = []

            # cut out thumbnail
            cvthumb = cv2.resize(color_frame[y:y+h,x:x+w],(64,64))
            face.thumb = opencv_bridge.cv2_to_imgmsg(cvthumb)

            # and publish
            self.face_pub.publish(face)


if __name__ == '__main__':

    # temp: wait until estimators are available
    time.sleep(3)

    # initialize node
    rospy.init_node('detect_faces')

    # initialize namespace hash
    ros_namespace_hash = hash(rospy.get_namespace())

    # create face detector object
    node = DetectFaces()

    # run
    rospy.spin()
