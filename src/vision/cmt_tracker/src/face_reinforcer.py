#!/usr/bin/env python2
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
    - If an cmt_tracker location has stayed in one location for quite a while and thus indication that it's may not be a face it's tracking.

Another is to destroy after a while if the tracker is going to garabage levels.

'''
class face_reinforcer:
    def __init__(self):
        rospy.init_node('face_reinforcer', anonymous=True)
        self.filtered_face_locations = rospy.get_param('filtered_face_locations')
        self.cmt_sub = message_filters.Subscriber('temporary_trackers',Trackers)
        self.face_sub = message_filters.Subscriber(self.filtered_face_locations, Objects)
        self.faces_cmt_overlap = {}

        ts = message_filters.ApproximateTimeSynchronizer([self.cmt_sub,self.face_sub], 10,0.1)
        ts.registerCallback(self.callback)

    def callback(self, cmt, face):
        '''
        This function aims to check the overlapping nature of the tracker and face. And if there is overlap increases the confidence of the tracker.

        This idea is that the trackers are always decaying and need to be reinforced to maintain tracking.
        @param cmt:
        @param face:
        @return:
        '''
        not_overlapped, overlaped_faces = self.returnOverlapping(face,cmt)

        for face, cmt in overlaped_faces:
            # Covered Faces this should be reinforcement. in another function.
            self.faces_cmt_overlap[cmt.tracker_name.data] = self.faces_cmt_overlap.get(cmt.tracker_name.data, 0) + 1

            if (self.faces_cmt_overlap[cmt.tracker_name.data] > 3):
                self.upt = rospy.ServiceProxy('validation',TrackerNames)
                indication = self.upt(names=cmt.tracker_name.data, index=int("0"))
                print("Updated to the main Tracker.")
                if not indication:
                    pass

    def returnOverlapping(self, face, cmt):
        '''
        This takes two messages the face message and the cmt_tracker Tracker message and checks the area between the two sets of areas and returns if there is an overlap.
        @param face:
        @param cmt:
        @return:
        '''
        print('starts overlapping')
        not_covered_faces = []
        overlaped_faces = []
        for j in face.objects:
            overlap = False
            SA = j.object.height * j.object.width
            for i in cmt.tracker_results:
                SB = i.object.object.height * i.object.object.width
                SI = max(0, (
                max(j.object.x_offset + j.object.width, i.object.object.x_offset + i.object.object.width) - min(
                    j.object.x_offset, i.object.object.x_offset))
                         * max(0, max(j.object.y_offset, i.object.object.y_offset) - min(
                    j.object.y_offset - j.object.height, i.object.object.y_offset - i.object.object.height)))
                SU = SA + SB - SI
                overlap_area = SI / SU
                overlap = overlap_area > 0
                if (overlap):
                    ## TODO DO we need to remove the tracker that was indeed needed in the element.
                    list = [j, i]
                    overlaped_faces.append(list)
                    break
            if not overlap:
                not_covered_faces.append(j)
        print('finishes overlapping')
        return not_covered_faces, overlaped_faces

if __name__ == '__main__':
    ic = face_reinforcer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass