#!/usr/bin/env python

from __future__ import division
from threading import Thread
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rospy
from io import BytesIO
import cv2
import os
import sys
import time
CWD = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(CWD, '../src'))

from vision.client import Client

hosts = ['http://localhost:9001', 'http://localhost:9002', 'http://localhost:9003', 'http://localhost:9004']
bridge = CvBridge()
client = Client(hosts)
image_cache = {}
processed_images = {}
start = time.time()
counter = 0
fps = 0
interval = 2

rospy.init_node('face_tracker')
imgpub = rospy.Publisher('~image', Image, latch=True, queue_size=1)


def callback(frame, response):
    global fps, counter, start
    now = time.time()
    counter += 1
    if (now-start) > interval:
        fps = counter/interval
        start = now
        counter = 0
    processed_images[frame] = response
    image = image_cache[frame]
    faces = response['faces']

    cv2.putText(image, 'FPS: {}'.format(fps), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 1)
    for left, top, right, bottom in faces:
        cv2.rectangle(image, (left, top), (right, bottom), (255, 0, 0), 1)

    all_landmarks = response.get('landmarks')
    if all_landmarks:
        for landmarks in all_landmarks:
            for x, y in landmarks:
                cv2.circle(image, (x,y), 1, (0, 255, 0), 1)

    imgpub.publish(bridge.cv2_to_imgmsg(image, 'bgr8'))
    del image_cache[frame]
    del processed_images[frame]

def cb(ros_image):
    frame = ros_image.header.seq
    image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    retval, buf = cv2.imencode('.jpeg', image, [cv2.IMWRITE_JPEG_QUALITY, 70])
    if retval:
        f = BytesIO(buf)
        client.detect_faces(frame, f, callback, landmarks=True)
        image_cache[frame] = image

rospy.Subscriber('/camera/image_raw', Image, cb)

rospy.spin()
