#!/usr/bin/env python

import os
import sys
import dlib
import rospy
from skimage import io
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import tempfile

class FaceDetector(object):

    def __init__(self, output_dir):
        self.face_detector = dlib.get_frontal_face_detector()
        self.bridge = CvBridge()
        if not os.path.isdir(output_dir):
            os.makedirs(output_dir)
        self.output_dir = tempfile.mkdtemp(dir=output_dir)
        self.count = 0

    def image_cb(self, ros_image):
        cv2_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        detected_faces = self.face_detector(cv2_image)
        self.count += 1
        for i, face_rect in enumerate(detected_faces):
            #print "Face #{} found at Left: {} Top: {} Right: {} Bottom: {}".format(i, face_rect.left(), face_rect.top(), face_rect.right(), face_rect.bottom())
            crop = cv2_image[face_rect.top():face_rect.bottom(), face_rect.left():face_rect.right()]
            if crop.size == 0:
                continue
            fname = os.path.join(self.output_dir, "cropped_{}_{}.jpg".format(self.count, i))
            cv2.imwrite(fname, crop)

if __name__ == '__main__':
    rospy.init_node("face_recognition")
    detecter = FaceDetector('faces')
    rospy.Subscriber('/camera/image_raw', Image, detecter.image_cb)
    rospy.spin()
