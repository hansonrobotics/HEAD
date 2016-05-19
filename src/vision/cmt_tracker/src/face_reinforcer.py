__author__ = 'tesfa'
import os
import sys
import subprocess
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import message_filters
from cmt_tracker_msgs.msg import Trackers,Tracker, Objects
from cmt_tracker_msgs.srv import TrackerNames

'''
Description: This is checks for the if cmt_tracker is overlapping with cmt instances and focuses the libraries to the locations.

Thus validates tracker to filter out values in the output of the tracker.
    - This before set's to avoid setting names or evaluating overlap before name is changed
    - This happens to avoid setting trackers for false positives in the tracker.

    Later: (Customization of the optical flow points into area that is being tracker bad. )
    - This also uses to set the area of the tracker to reset the tracker if it diverges a lot.

Another is to destroy after a while if the tracker is going to garabage levels.

'''
class face_reinforcer:
    def __init__(self):

        self.camera_topic = rospy.get_param('camera_topic')
        self.filtered_face_locations = rospy.get_param('filtered_face_locations')

        self.image_sub = message_filters.Subscriber(self.camera_topic, Image)
        self.cmt_sub = message_filters.Subscriber('tracker_results',Trackers)
        self.face_sub = message_filters.Subscriber(self.filtered_face_locations, Objects)

        ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.cmt_sub,self.face_sub], 10,0.1)

        ts.registerCallback(self.callback)

    def callback(self, data, cmt, face):
        try:
          cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
          print(e) # This need to handle the possibility of errors.

        # Now the overlap has to be measured
        # Now another things to do is to do a relative overlap the area.
        for j in face.objects:
            SA = j.object.height * j.object.width
            overlap = False
            tupl = []
            for i in cmt.tracker_results:
                SB = i.object.object.height * i.object.object.width
                #SI = max(0, max(XA2, XB2) - min(XA1, XB1)) * max(0, max(YA2, YB2) - min(YA1, YB1))
                SI = max(0, ( max(j.object.x_offset + j.object.width,i.object.object.x_offset + i.object.object.width)- min(j.object.x_offset,i.object.object.x_offset) )
                         * max(0,max(j.object.y_offset,i.object.object.y_offset) - min(j.object.y_offset - j.object.height,i.object.object.y_offset - i.object.object.height)))
                # print("Face is: %s", x)

                SU = SA + SB - SI

                overlap_area = SI / SU
                # print(SU)

                overlap = overlap_area > 0