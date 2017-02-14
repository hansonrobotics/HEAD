#!/usr/bin/env python

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rospy
from io import BytesIO
import cv2
import os
import sys
CWD = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(CWD, '../src'))

from vision.client import Client

bridge = CvBridge()
client = Client()

rospy.init_node('face_tracker')
imgpub = rospy.Publisher('~image', Image, latch=True, queue_size=1)

def cb(ros_image):
    image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    retval, buf = cv2.imencode('.png', image)
    f = BytesIO(buf)
    response = client.detect_faces(f)
    faces = response['faces']

    for left, top, right, bottom in faces:
        cv2.rectangle(image, (left, top), (right, bottom), (255, 0, 0), 2)
    imgpub.publish(bridge.cv2_to_imgmsg(image, 'bgr8'))

rospy.Subscriber('/camera/image_raw', Image, cb)

rospy.spin()
