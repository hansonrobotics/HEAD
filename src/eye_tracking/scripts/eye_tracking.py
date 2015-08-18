#!/usr/bin/env python

import rospy
import cv2
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class EyeTracking:
    def __init__(self):
        # Node parameters
        self.topic = rospy.get_param("~topic", "camera/image_raw")
        self.angle = rospy.get_param("~angle", 0)
        self.scale = rospy.get_param("~scale", 0.1)
        self.crop = rospy.get_param("~crop", 0.1)

        # cascade for face detection
        self.cascade = cv2.CascadeClassifier(os.path.join(
            os.path.dirname(os.path.realpath(__file__)), "haarcascade.xml"))
        # Bridge to convert images to OpenCV
        self.bridge = CvBridge()
        # subscribe camera topic
        rospy.Subscriber(self.topic, Image, self.camera_callback)

    def camera_callback(self,img):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError, e:
            print e
            return

        # pre-processing
        self.rotate()
        self.image = self.cv_image.copy()
        self.resize()
        # detect faces
        faces = self.faces()
        for f in faces:
            face = [int(x/self.scale) for x in f]
            cv2.rectangle(self.image, (face[0],face[1]),(face[0]+face[2],face[1]+face[3]), (255,0,0), 3)

        cv2.imshow("Eye View", self.image)
        cv2.waitKey(1)

    def rotate(self):
        rows,cols = (self.cv_image.shape)[:2]

        M = cv2.getRotationMatrix2D((cols/2,rows/2),self.angle,1)
        self.cv_image = cv2.warpAffine(self.cv_image, M, (cols,rows))

    def resize(self):
        # crop image after rotation
        rows, cols = (self.cv_image.shape)[:2]
        crop_rows = round(rows*self.crop)
        crop_cols = round(cols*self.crop)
        self.cv_image = self.cv_image[crop_rows:-crop_rows, crop_cols:-crop_cols]
        # resize
        self.cv_image = cv2.resize(self.cv_image, (cols/2, rows/2), interpolation=cv2.INTER_AREA)


    def faces(self):
        gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)

        faces = self.cascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30),
            flags=cv2.cv.CV_HAAR_SCALE_IMAGE
        )
        return faces


if __name__ == '__main__':
    rospy.init_node('eye_tracking')
    ET = EyeTracking()
    rospy.spin()

