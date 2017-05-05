#!/usr/bin/env python2.7
import os
import rospy
import logging
import time
import cv2

from sensor_msgs.msg import Image;
from fusion1.msg import Face,Thumb;
from cv_bridge import CvBridge;

from std_msgs.msg import String;

logger = logging.getLogger('hr.fusion1')


global_uid = 0


def GenerateUID():
    global global_uid
    result = global_uid
    global_uid += 1
    return result


class DetectFaces(object):

    def __init__(self):

        # initialize face detection
        self.opencv_bridge = CvBridge()
        self.face_cascade = cv2.CascadeClassifier("/home/desmond/hansonrobotics/HEAD/src/fusion1/test/haarcascade_frontalface_alt.xml")
        if self.face_cascade.empty():
            print "cascade not found"

        # initialize current image and timestamp
        self.current_image = Image()
        self.current_ts = 0.0

        # initialize face detection period
        self.period_sec = 0.2 # 5Hz

        # start subscribers and publishers
        self.image_sub = rospy.Subscriber("camera/image_raw",Image,self.HandleImage)
        self.face_pub = rospy.Publisher("face",Face,queue_size=5)
        self.thumb_pub = rospy.Publisher("thumb",Thumb,queue_size=5)

        # start timer
        self.timer = rospy.Timer(rospy.Duration(self.period_sec),self.HandleTimer)

        # for debugging
        #cv2.startWindowThread()
        #cv2.namedWindow("detect_faces")


    def HandleImage(self,data):

        # when a camera frame comes in, just store it as current image, as well as the time
        # this is the time that the robot actually saw something

        # overwrite current image and timestamp
        self.current_image = data
        self.current_ts = rospy.get_rostime()


    def HandleTimer(self,event):

        # at regular intervals, perform face detection and publish those observations over ROS

        # make sure there is an image to process
        if self.current_ts == 0.0:
            return

        # get image dimensions
        xsize = self.current_image.width
        ysize = self.current_image.height

        # detect faces
        gray = self.opencv_bridge.imgmsg_to_cv2(self.current_image,"mono8")
        faces = self.face_cascade.detectMultiScale(gray,scaleFactor=1.1,minSize=(30,30),flags=cv2.cv.CV_HAAR_SCALE_IMAGE)

        # for debugging
        #for (x,y,w,h) in faces:
        #    cv2.rectangle(gray, (x,y), (x+w,y+h), 255 , 1)
        #cv2.imshow("detect_faces",gray)

        # don't publish anything if there were no faces detected
        if len(faces) == 0:
            return

        # TODO: for each face, discover landmarks

        # publish faces
        for (x,y,w,h) in faces:

            # create new UID for the face
            uid = GenerateUID()

            # publish thumbnail
            pub_thumb = Thumb()
            pub_thumb.ts = self.current_ts
            pub_thumb.uid = uid
            pub_thumb.image = self.opencv_bridge.cv2_to_imgmsg(cv2.resize(gray[y:y+h,x:x+w],(32,32)))
            self.thumb_pub.publish(pub_thumb)

            # publish face data
            cx = x + w / 2
            cy = y + h / 2
            pub_face = Face()
            pub_face.ts = self.current_ts
            pub_face.uid = uid
            pub_face.rect.center.x = -1.0 + 2.0 * float(cx) / float(xsize)
            pub_face.rect.center.y = -1.0 + 2.0 * float(cy) / float(ysize)
            pub_face.rect.size.x = 2.0 * float(w) / float(xsize)
            pub_face.rect.size.y = 2.0 * float(h) / float(ysize)
            pub_face.landmarks = []
            self.face_pub.publish(pub_face)


if __name__ == '__main__':

    # initialize node
    rospy.init_node('detect_faces')
    node = DetectFaces()

    # run
    rospy.spin()
