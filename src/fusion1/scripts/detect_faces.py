#!/usr/bin/env python2.7
import os
import rospy
import numpy
import time
import cv2
import dynamic_reconfigure.client
from sensor_msgs.msg import Image
from fusion1.msg import Face
from cv_bridge import CvBridge


opencv_bridge = CvBridge()
face_cascade = cv2.CascadeClassifier(os.getenv("HOME") + "/hansonrobotics/HEAD/src/fusion1/test/haarcascade_frontalface_alt.xml")
if face_cascade.empty():
    print "cascade not found"


serial_number = 0
def GenerateFaceID():
    global serial_number
    result = serial_number
    serial_number += 1
    return result


class DetectFaces(object):


    def __init__(self):

        self.cur_frame = Image()
        self.cur_ts = 0.0

        rospy.wait_for_service("vision_pipeline")
        self.dynparam = dynamic_reconfigure.client.Client("vision_pipeline",timeout=30,config_callback=self.HandleConfig)        
        self.fovy = rospy.get_param("fovy")
        self.aspect = rospy.get_param("aspect")
        self.rate = rospy.get_param("rate")

        self.image_sub = rospy.Subscriber("camera/image_raw",Image,self.HandleImage)
        self.face_pub = rospy.Publisher("face",Face,queue_size=5)

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate),self.HandleTimer)


    def HandleConfig(self,data):

        print "detect_faces {:?}".format(data)


    def HandleImage(self,data):

        self.cur_image = data
        self.cur_ts = rospy.get_rostime()


    def HandleTimer(self,data):

        if self.cur_ts == 0.0:
            return

        color_image = opencv_bridge.imgmsg_to_cv2(self.cur_image,"bgr8")
        faces = face_cascade.detectMultiScale(color_image,scaleFactor=1.1,minSize=(30,30),flags=cv2.cv.CV_HAAR_SCALE_IMAGE)

        if len(faces) == 0:
            return

        for (x,y,w,h) in faces:

            if (w <= 0) or (h <= 0):
                continue

            face = Face()
            face.face_id = GenerateFaceID()
            face.ts = self.cur_ts
            face.rect.origin.x = -1.0 + 2.0 * float(x) / float(self.cur_image.width)
            face.rect.origin.y = -1.0 + 2.0 * float(y) / float(self.cur_image.height)
            face.rect.size.x = 2.0 * float(w) / float(self.cur_image.width)
            face.rect.size.y = 2.0 * float(h) / float(self.cur_image.height)
            face.pos.x = face.rect.origin.x + face.rect.size.x / 2
            face.pos.y = face.rect.origin.y + face.rect.size.y / 2
            face.pos.z = 1.0
            face.confidence = 1.0
            face.smile = 0.0
            face.frown = 0.0
            face.expressions = []
            face.landmarks = []
            cvthumb = cv2.resize(color_image[y:y+h,x:x+w],(64,64))
            face.thumb = opencv_bridge.cv2_to_imgmsg(cvthumb,encoding="8UC3")

            self.face_pub.publish(face)


if __name__ == '__main__':

    rospy.init_node('detect_faces')
    node = DetectFaces()
    rospy.spin()
