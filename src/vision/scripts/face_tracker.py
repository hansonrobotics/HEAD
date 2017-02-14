#!/usr/bin/env python

from threading import Thread
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

hosts = ['http://localhost:9001', 'http://localhost:9002', 'http://localhost:9003', 'http://localhost:9004']
bridge = CvBridge()
client = Client(hosts)
image_cache = {}
processed_images = {}

rospy.init_node('face_tracker')
imgpub = rospy.Publisher('~image', Image, latch=True, queue_size=1)

def callback(frame, response):
    processed_images[frame] = response
    image = image_cache[frame]
    faces = response['faces']
    for left, top, right, bottom in faces:
        cv2.rectangle(image, (left, top), (right, bottom), (255, 0, 0), 2)
    imgpub.publish(bridge.cv2_to_imgmsg(image, 'bgr8'))
    del image_cache[frame]
    del processed_images[frame]

def cb(ros_image):
    frame = ros_image.header.seq
    image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    retval, buf = cv2.imencode('.png', image)
    f = BytesIO(buf)
    client.detect_faces(frame, f, callback)
    image_cache[frame] = image

rospy.Subscriber('/camera/image_raw', Image, cb)

rospy.spin()
