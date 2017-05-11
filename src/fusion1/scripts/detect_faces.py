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
ros_namespace_hash = 0
serial_number = 0

def GenerateID():
    global ros_namespace_hash
    global serial_number
    result = ros_namespace_hash + serial_number
    serial_number += 1
    return result


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
        gray_frame = opencv_bridge.imgmsg_to_cv2(self.most_recent_frame,"mono8")
        faces = face_cascade.detectMultiScale(gray_frame,scaleFactor=1.1,minSize=(30,30),flags=cv2.cv.CV_HAAR_SCALE_IMAGE)

        # don't publish anything if there were no faces detected
        if len(faces) == 0:
            return

        # publish all faces
        for (x,y,w,h) in faces:

            # prepare face message
            face = Face()

            # create new UID for the face
            face.id = GenerateID()

            # set timestamp
            face.ts = self.most_recent_ts

            # TODO: convert rectangle to 3D coordinates
            face.x = -1.0 + 2.0 * float(x + w / 2) / float(self.most_recent_frame.width)
            face.y = -1.0 + 2.0 * float(y + h / 2) / float(self.most_recent_frame.height)
            face.z = 1.0

            # assume it's a valid face
            face.confidence = 1.0

            # cut out thumbnail (a little larger, so OpenBR can correctly identify stuff)
            tx = x - 10
            ty = y - 10
            tw = w + 20
            th = h + 20
            face.image = opencv_bridge.cv2_to_imgmsg(cv2.resize(gray_frame[ty:ty+th,tx:tx+tw],(64,64)))

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
