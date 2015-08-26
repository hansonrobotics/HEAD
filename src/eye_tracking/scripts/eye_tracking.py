#!/usr/bin/env python

import rospy
import cv2
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from ros_pololu.msg import MotorCommand
from pau2motors.msg import pau
from topic_tools.srv import MuxSelect
import time

class EyeTracking:
    def __init__(self):
        # Eye Tracking enabled for the node
        self.tracking_params = rospy.get_param("eye_tracking", False)
        if not self.tracking_params:
            rospy.signal_shutdown("No Params set")
        # Node parameters for processing image
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
        # Where eyes should be looking at (relative w and h of the bounding box). Center by default.
        # This should be updated from behavior tree or procedural animations
        self.target = [0.3, 0.3]
        # Eye tracking parameters
        self.tracking_params = rospy.get_param("eye_tracking")
        # Distance which will need to be adjusted to closest face. Relative to picture size.
        self.face_distance = [0, 0]
        # Publishing motor messages disabled by default
        self.publishing = rospy.get_param("~autostart", True)
        # Pau messages
        self.pau_pub = rospy.Publisher("eyes_tracking_pau", pau, queue_size=10)
        # Subscribe PAU from eyes
        self.pau_sub = rospy.Subscriber("/blender_api/get_pau", pau, self.pau_callback)
        rospy.wait_for_service("eyes_pau_mux/select")
        self.pau_ser = rospy.ServiceProxy("eyes_pau_mux/select", MuxSelect)
        # Angles to added already
        self.added = {'w': 0, 'h': 0}
        if self.publishing:
            self.pau_ser.call("eyes_tracking_pau")

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
        # find face in middle
        face = self.closest_face(faces)
        if face is None:
            self.face_distance = [0,0]
        else:
            self.face_distance = self.distance(face)
        # Draw faces on image
        for f in faces:
            face = [int(x/self.scale) for x in f]
            cv2.rectangle(self.image, (face[0],face[1]),(face[0]+face[2],face[1]+face[3]), (255,0,0), 3)
        rows,cols = (self.image.shape)[:2]
        center = (int(cols*self.tracking_params['center']['w']),int(rows*self.tracking_params['center']['h']))
        cv2.circle(self.image,center, 3,(0,255,0))
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
        self.im_w  = int(cols * self.scale)
        self.im_h =  int(rows * self.scale)
        self.cv_image = cv2.resize(self.cv_image, (self.im_w, self.im_h), interpolation=cv2.INTER_AREA)

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
    # Finds face distance from camera to target of where to look at
    def distance(self, f):
        return [(f[0] + (f[2])*self.target[0])/self.im_w - self.tracking_params['center']['w'],
                (f[1] + (f[3])*self.target[1])/self.im_h - self.tracking_params['center']['h']]

    def closest_face(self, faces):
        min_distance = self.tracking_params['distance-max']
        face = None
        for f in faces:
            d = self.distance(f)
            # linear distance
            d = (d[0]**2+d[1]**2)**0.5
            if d <= min_distance:
                face = f
                min_distance = d
        return face

    def pau_callback(self, msg):
        if self.face_distance != [0,0]:
            dw = self.face_distance[0]*self.tracking_params['angles']['w']
            dh = self.face_distance[1]*self.tracking_params['angles']['h']
        else:
            # Gradually remove correction
            dw = -self.added['w']*0.03
            dh = -self.added['h']*0.03

        self.added['w'] += dw
        self.added['h'] += dh
        # reset distance
        self.face_distance = [0,0]
        # foward to pau_topic adjusted angles
        msg.m_eyeGazeLeftPitch -= self.added['h']
        msg.m_eyeGazeRightPitch -= self.added['h']
        msg.m_eyeGazeLeftYaw += self.added['w']
        msg.m_eyeGazeRightYaw += self.added['w']
        self.pau_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('eye_tracking')
    ET = EyeTracking()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        r.sleep()