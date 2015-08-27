#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
topic = '/camera/image_raw'
fps = 20

def publish_video(file):
    cap = cv2.VideoCapture(file)
    r = rospy.Rate(fps)
    bridge = CvBridge()
    i = 0
    frames = cap.get(cv2.cv.CV_CAP_PROP_FRAME_COUNT)
    pub = rospy.Publisher(topic, Image, queue_size=10)
    while(cap.isOpened() and not rospy.is_shutdown()):
        ret, frame = cap.read()
        i+=1
        if ret:
            msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            pub.publish(msg)
        if i == frames:
            cap = cv2.VideoCapture(file)
            i = 0
        r.sleep()
    cap.release()

if __name__ == '__main__':
    rospy.init_node('video_publisher')
    file = rospy.get_param("~file")
    topic = rospy.get_param("~topic",topic)
    fps = rospy.get_param("~fps",fps)
    publish_video(file)


