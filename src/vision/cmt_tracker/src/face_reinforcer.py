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
from std_srvs.srv import Empty
from cmt_tracker_msgs.msg import Trackers,Tracker, Objects
from cmt_tracker_msgs.srv import TrackerNames,MergeNames

import itertools
import logging
from dynamic_reconfigure.server import Server
from cmt_tracker_msgs.cfg import ReinforceConfig
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
        self.tracker_locations_pub = rospy.Publisher("tracking_locations", Trackers, queue_size=5)
        self.cmt_sub_ = message_filters.Subscriber('tracker_results', Trackers)
        self.face_sub = message_filters.Subscriber(self.filtered_face_locations, Objects)
        self.faces_cmt_overlap = {}
        self.srvs = rospy.Service('can_add_tracker', Empty, self.can_update)

        ts = message_filters.ApproximateTimeSynchronizer([self.cmt_sub,self.face_sub,self.cmt_sub_], 1,0.2)
        ts.registerCallback(self.callback)

        self.update = True

        #Last x amount of results of the centroid of the rect that opencv has published.
        #If there is indeed a face then we need to create a tracker for it. It wouldn't work for face_recogniztion but
        #Would work nonethless.
        #So is's centroid area, decreasing conter; then remove that entity.
        # logging.getLogger().addHandler(logging.StreamHandler())
        self.logger = logging.getLogger('hr.cmt_tracker.face_reinforcer')
        Server(ReinforceConfig, self.sample_callback)

        self.persistance_face = []
        self.face = False
        self.persistance_cv = False
        self.persistance_dlib = False

        self.dlib_count = 4
        self.cv_count = 6
        self.cv_dlib_count = 1
        self.downgrade = 500

    def sample_callback(self,config, level):
        self.dlib_count = config.dlib_count
        self.cv_count = config.open_count
        self.cv_dlib_count = config.open_dlib_count
        self.downgrade = config.downgrade
        return config

    def can_update(self, req):
        self.update = True
        return []

    def callback(self, cmt, face,temp):

        ttp = cmt
        for val in temp.tracker_results:
            ttp.tracker_results.append(val)

        not_overlapped, overlaped_faces = self.returnOverlapping(face,ttp)
        if len(not_overlapped) > 0 and self.update:
            self.tracker_locations_pub.publish(self.convert(not_overlapped))
            #Executing the following doesn't really do anythin as it just published it.
            #self.view_update = rospy.ServiceProxy('view_update', TrackerNames)
            #updt = self.view_update(names="", index=1)
            self.update = False

        for face, cmt in overlaped_faces:

            self.faces_cmt_overlap[cmt.tracker_name.data] = self.faces_cmt_overlap.get(cmt.tracker_name.data, 0) + 2
            if (self.faces_cmt_overlap[cmt.tracker_name.data] > 2):
                try:
                    self.upt = rospy.ServiceProxy('reinforce',TrackerNames)
                    indication = self.upt(names=cmt.tracker_name.data, index=self.downgrade)
                    if not indication:
                        #TODO handle the error if the service is not available.
                        pass
                except rospy.ServiceException, e:
                    self.logger.error("Reinforcing Service call failed: %s" % e)

        # TODO if the cmt name has disappeared then remove it from the self.faces_cmt_overlap.get Via Subscriber to Face_events
        self.cmt_merge = {}
        merge_to=[]
        merge_from=[]
        for a, b in itertools.combinations(ttp.tracker_results, 2):
            SA = a.object.object.height * a.object.object.width
            SB = b.object.object.height * b.object.object.width
            SI = max(0, (
                max(a.object.object.x_offset + a.object.object.width, b.object.object.x_offset + b.object.object.width) - min(
                    a.object.object.x_offset, b.object.object.x_offset))
                         * max(0, max(a.object.object.y_offset, b.object.object.y_offset) - min(
                    a.object.object.y_offset - a.object.object.height, b.object.object.y_offset - b.object.object.height)))
            SU = SA + SB - SI
            if (SU != 0):
                overlap_area_ = SI / SU
            else:
                overlap_area_ = 1

            overlap_ = overlap_area_ > 0
            if (overlap_):
                #TODO Choose which to merge too later information to which it's closer too.
                self.cmt_merge[a.tracker_name.data] = self.cmt_merge.get(a.tracker_name.data,b.tracker_name.data)
                merge_to.append(a.tracker_name.data)
                merge_from.append(b.tracker_name.data)

        #TODO for now just delete the elements then latter put a mark on the merged elements and update if there is overlappings with the track.
        if merge_to and merge_from:
            try:
                print("Merged Element: %s and \n Merged ELement %s", merge_from, merge_to)
                self.mrg = rospy.ServiceProxy('merge',MergeNames)
                indic =self.mrg(merge_to=merge_to, merge_from=merge_from)

                if not indic:
                    pass
            except rospy.ServiceException, e:
                self.logger.error("Merging Service call failed: %s" % e)

        #Now pass to the merger.



    def returnOverlapping(self, face, cmt):
        not_covered_faces_list = []
        not_covered_faces = []
        overlaped_faces = []

        for get_element in self.persistance_face:
            get_element[4] += 1


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
                if SU != 0:
                    overlap_area = SI / SU
                else:
                    overlap_area = 1
                overlap = overlap_area > 0
                if (overlap):
                    list = [j, i]
                    overlaped_faces.append(list)
                    break

            if not overlap:
                not_covered_faces_list.append(j)

            #This section is for creating tracker locations by updating not_covered_faces
        for j in not_covered_faces_list:
            if (j.object.width * j.object.height > 0.4*(640*480)):
                print('Face To Big to added')
                break
            overlp = False
            for get_element in self.persistance_face:
                overlp = self.determine(get_element, j)
                if overlp:
                    if j.tool_used_for_detection.data == "dlib":
                        get_element[5] += 1
                    else:
                        get_element[6] += 1
                    if get_element[5] > self.dlib_count:
                        not_covered_faces.append(j)
                        self.persistance_face=[]
                    elif get_element[6] > self.cv_count:
                        not_covered_faces.append(j)
                        self.persistance_face=[]
                    # elif get_element[6] + get_element [5] > self.cv_dlib_count:
                    #     not_covered_faces.append(j)
                    #     self.persistance_face=[]
            if not overlp:
                self.persistance_face.append(
                        [j.object.x_offset, j.object.width, j.object.y_offset, j.object.height, 0,
                         1 if j.tool_used_for_detection == "dlib" else 0,
                         1 if j.tool_used_for_detection != "dlib" else 0])

        self.persistance_face[:] = [get_element for get_element in self.persistance_face if not (self.trim(get_element))]

        print("++++++++++++++++")
        print(self.dlib_count)
        print(self.cv_count)
        print("++++++++++++++++")

        return not_covered_faces, overlaped_faces
    def determine(self, get_element,j):
        epsilon_x = 15
        epsilon_y = 15

        x_off = (j.object.x_offset)
        y_off = j.object.y_offset

        gx_off =get_element[0]
        gy_off =get_element[2]

        x_res = abs(x_off - gx_off)
        y_res = abs(y_off - gy_off)

        w_res = abs(j.object.width - get_element[1])
        h_res = abs(j.object.height - get_element[3])

        if x_res < epsilon_x and y_res < epsilon_y and w_res < epsilon_y and h_res < epsilon_y:
            return True
        return False

    def trim(self,get_element):
        if (get_element[4] < 10):
            return False
        return True

    def convert(self, face_locs, opencv=False):
        message = Trackers()
        for i in face_locs:
            messg = Tracker()
            messg.object = i
            message.tracker_results.append(messg)
        message.header.stamp = rospy.Time.now()
        return message

if __name__ == '__main__':
    ic = face_reinforcer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass