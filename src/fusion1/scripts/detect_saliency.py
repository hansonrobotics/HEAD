#!/usr/bin/env python2.7
import os
import rospy
import numpy
import time
import cv2
import dynamic_reconfigure.client
from sensor_msgs.msg import Image
from fusion1.msg import Saliency,Float32XYZ
from cv_bridge import CvBridge


opencv_bridge = CvBridge()

subx = 320
suby = 240

covx = 128
covy = 96


serial_number = 0
def GenerateSaliencyID():
    global serial_number
    result = serial_number
    serial_number += 1
    return result


class DetectSaliency(object):

    def __init__(self):

        self.cur_image = Image()
        self.last_image = Image()
        self.cur_ts = 0.0
        self.last_ts = 0.0

        rospy.wait_for_service("vision_pipeline")
        self.dynparam = dynamic_reconfigure.client.Client("vision_pipeline",timeout=30,config_callback=self.HandleConfig)        
        self.fovy = rospy.get_param("fovy")
        self.aspect = rospy.get_param("aspect")
        self.rate = rospy.get_param("rate")

        self.image_sub = rospy.Subscriber("camera/image_raw",Image,self.HandleImage)
        self.saliency_pub = rospy.Publisher("saliency",Saliency,queue_size=5)
        self.control_sub = rospy.Subscriber("vision_control",VisionControl,self.HandleVisionControl)

        self.timer = rospy.Timer(rospy.Duration(self.period),self.HandleTimer)

        # for debugging
        #cv2.startWindowThread()
        #cv2.namedWindow("faux saliency")
        #self.total_image = numpy.zeros((suby,subx),numpy.uint8)


    def HandleConfig(self,data):

        print "detect_saliency {:?}".format(data)


    def HandleImage(self,data):

        self.last_image = self.cur_image
        self.last_ts = self.cur_ts
        self.cur_image = data
        self.cur_ts = rospy.get_rostime()


    def HandleTimer(self,data):

        if self.cur_ts == 0.0:
            return

        # the algorithm is roughly based to the work of Itti & Koch in the early 2000s

        # there are major artistic reductions for OpenCV2+Python+Hanson+Sophia context

        # the solution can be viewed as a saliency-detecting CNN,
        # but with handcrafted weights via traditional vision operations

        # convert ROS images to OpenCV and rescale to subx,suby, the working resolution
        # also, convert from BGR to YUV (which is more natural for vision tasks)
        bgr_cur_image = opencv_bridge.imgmsg_to_cv2(self.cur_image)
        yuv_u8 = cv2.cvtColor(cv2.resize(bgr_cur_image,(subx,suby),interpolation=cv2.INTER_LINEAR),cv2.COLOR_BGR2YUV)
        yuv = yuv_u8.astype("float32")

        bgr_last_image = opencv_bridge.imgmsg_to_cv2(self.last_image)
        yuv_last_u8 = cv2.cvtColor(cv2.resize(bgr_last_image,(subx,suby),interpolation=cv2.INTER_LINEAR),cv2.COLOR_BGR2YUV)
        yuv_last = yuv_last_u8.astype("float32")

        # generate motion map from difference between last frame and current frame
        dyuv = cv2.subtract(yuv,yuv_last)

        # split intensity and color maps
        yc,uc,vc = cv2.split(yuv) 
        dyc,duc,dvc = cv2.split(dyuv)

        total = numpy.zeros((suby,subx),numpy.float)

        # use Gaussian blur to create low-pass filtered versions of each map
        yc4 = cv2.GaussianBlur(yc,(13,13),0) # intensity
        uc4 = cv2.GaussianBlur(uc,(13,13),0) # red-green
        vc4 = cv2.GaussianBlur(vc,(13,13),0) # blue-yellow
        dyc4 = cv2.GaussianBlur(dyc,(13,13),0) # motion

        # emulate high-pass filtering (high-frequency content is the 'most interesting') by
        # subtracting low-pass images from unfiltered images

        # furthermore, for the non-motion maps, use Laplacian as optical derivative

        # generally, motion is the 'most interesting',
        total += (dyc - dyc4) * 6.0

        # color responses are 'a bit less interesting',
        total += cv2.Laplacian(vc - vc4,-1) * 3.0
        total += cv2.Laplacian(uc - uc4,-1) * 3.0

        # intensity contrast is 'also somewhat interesting'
        total += cv2.Laplacian(yc - yc4,-1)

        # normalize and take absolute value (so negative responses are also taken into account)
        dst = total.copy()
        cv2.normalize(cv2.absdiff(total,0.0),dst,0.0,1.0,cv2.NORM_MINMAX)

        # find 5 successive brightest points in a lower resolution result
        resized = cv2.resize(dst,(covx,covy),interpolation=cv2.INTER_LINEAR)
        points = []
        for i in range(0,5):
            brightest_v = 0.0
            point = Float32XYZ()
            for y in range(0,covy):
                for x in range(0,covx):
                    if resized[y,x] > point.z:
                        point.x = x
                        point.y = y
                        point.z = resized[y,x]
            points.append(point)
            cv2.circle(resized,(point.x,point.y),4,0,-1)

        # convert to messages and send off

        # the idea is that these messages signal the existance of salient points in the camera image to the fusion layer
        # there, a collection of possibly salient directions is maintained and tracked, like the faces

        # later on, the AI can select which of these salient directions is interesting to watch,
        # and have the robot cycle through them at random; the robot will then appear to be glancing
        # at interesting things
        for point in points:
            saliency = Saliency()
            saliency.saliency_id = GenerateSaliencyID()
            saliency.ts = self.cur_ts
            saliency.pos.x = -1.0 + 2.0 * (point.x / float(covx))
            saliency.pos.y = -1.0 + 2.0 * (point.y / float(covy))
            saliency.confidence = point.z
            self.saliency_pub.publish(saliency)

        #cv2.imshow("faux saliency",cv2.resize(resized,(subx,suby),interpolation=cv2.INTER_LINEAR))

if __name__ == '__main__':

    rospy.init_node('detect_saliency')
    node = DetectSaliency()
    rospy.spin()
