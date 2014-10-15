#!/usr/bin/env python

""" face_tracker.py - Version 0.22 2012-01-20

    Track a face using the OpenCV Haar detector to initially locate the face, then OpenCV's
    Good-Features-to-Track and Lucas-Kanade Optical Flow to track the face features over 
    subsequent frames.
    
    Can also be used to track arbitrarily selected patches by setting the parameter
    auto_face_tracking to False and using the mouse to select the desired region.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2011 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import roslib
roslib.load_manifest('pi_face_tracker')
import rospy
import cv
import sys
from sensor_msgs.msg import RegionOfInterest, Image
from math import sqrt, isnan
from ros2opencv import ROS2OpenCV
from pi_face_tracker.srv import *
import time

''' Single face area class '''
class FaceBox():

    #top left, and right bottom corner points given
    def __init__(self, id, pt1, pt2):
        # unique id for this session
        self.face_id = id
        #parameters to adjust for better detection
        self.min_age = 5 # need at least 5 frames to make it new

        self.min_area = 0.5 #coeff of how much area of the detected box should overlap in order to make same judgement
        self.age = 0
        self.skipped = 0 #frames that couldn't find the face
        self.valid = False
        # available
        self.status = "init"
        self.pt1 = pt1 # (x1,y1)
        self.pt2 = pt2 # (x2,y2)
        self.terminated = False
        self.features = []
        self.expand_roi = 1.02
        self.track_box = None
        self.min_features = 50
        self.abs_min_features = 6

    def area(self):
        return (self.pt2[0]-self.pt1[0])*(self.pt2[1]-self.pt1[1])

    def valid(self):
        if (self.age < self.min_age):
            return False
        return True
    def _overlap_area(self,pt1,pt2):
        dx = max(0,min(self.pt2[0],pt2[0])-max(self.pt1[0],pt1[0]))
        dy = max(0,min(self.pt2[1],pt2[1])-max(self.pt1[0],pt1[1]))
        return dx*dy

    def is_same_face(self,pt1,pt2):
        if self.terminated:
            return False

        overlap = self._overlap_area(pt1,pt2)
        if (overlap/self.area() > self.min_area):
            self.pt1 = pt1
            self.pt2 = pt2
            # reset frames skipped
            self.skipped = 0
            return True
        return False

    def next_frame(self):
        self.age += 1
        self.skipped += 1
        if self.status == 'init':
            if self.age - self.skipped >= self.min_age:
                self.status = 'new'

    def get_box(self):
        return [self.pt1,self.pt2]

    def face_box(self):
        face_width = self.pt2[0] - self.pt1[0]
        face_height = self.pt2[1] - self.pt1[1]
        face_box = (self.pt1[0], self.pt1[1], face_width, face_height)
        return face_box

    def update_box(self,box):
        self.track_box = box
        if not box is None:
            x,y,w,h = box
            self.pt1 = (x,y)
            self.pt2 = (x+w, y+h)

    def update_box_elipse(self, elipse):
        self.track_box = elipse
        if not elipse is None:
            (roi_center, roi_size, roi_angle) = elipse
            self.pt1 = (int(roi_center[0] - roi_size[0]/2), int(roi_center[1] - roi_size[1]/2))
            self.pt2 = (int(roi_center[0] + roi_size[0]/2), int(roi_center[1] + roi_size[1]/2))


class FacesRegistry():
    def __init__(self):
        self.step = 0
        self.face_id = 0
        self.faces = {}


    def nextFrame(self):
        self.step += 1

    ''' Faces array of (pt1,pt2) '''
    def addFaces(self,faces):
        for f in faces:
            if not self.faces:
                self.face_id += 1
                self.faces[self.face_id] = FaceBox(self.face_id,f[0], f[1])
            else:
                for id in self.faces.keys():
                    if self.faces[id].is_same_face(f[0], f[1]):
                        break
                    self.face_id += 1
                    self.faces[self.face_id] = FaceBox(self.face_id,f[0], f[1])

    #processes faces statuses:
    # Calls new_face callback if new faces are added,
    # Calls onExit when face
    def checkStatus(self, new_face = None, on_exit = None):
        for id in self.faces.keys():
            #check new
            f = self.faces[id]
            if f.status == 'new':
                f.status = 'ok'
                if new_face is not None:
                    new_face(f)
            if f.status == 'terminated':
                if on_exit is not None:
                    on_exit(f)
                del self.faces[id]

    #returns Face by id. If no id given returns oldest face in scene
    def getFace(self, id=-1):
        if id < 0:
            for i in self.faces.keys():
                if self.faces[i].valid():
                    return self.faces[i]
            return False
        if id in self.faces:
            # may need to check if valid
            return self.faces[id]
        return None

class PatchTracker(ROS2OpenCV):
    def __init__(self, node_name):
        ROS2OpenCV.__init__(self, node_name)
        
        self.node_name = node_name       
        
        self.auto_face_tracking = rospy.get_param("~auto_face_tracking", True)
        self.use_haar_only = rospy.get_param("~use_haar_only", False)
        self.use_depth_for_detection = rospy.get_param("~use_depth_for_detection", False)
        self.fov_width = rospy.get_param("~fov_width", 1.094)
        self.fov_height = rospy.get_param("~fov_height", 1.094)
        self.max_face_size = rospy.get_param("~max_face_size", 0.28)
        self.use_depth_for_tracking = rospy.get_param("~use_depth_for_tracking", False)
        self.auto_min_features = rospy.get_param("~auto_min_features", True)
        self.min_features = rospy.get_param("~min_features", 50) # Used only if auto_min_features is False
        self.abs_min_features = rospy.get_param("~abs_min_features", 6)
        self.std_err_xy = rospy.get_param("~std_err_xy", 2.5)
        self.pct_err_z = rospy.get_param("~pct_err_z", 0.42) 
        self.max_mse = rospy.get_param("~max_mse", 10000)
        self.good_feature_distance = rospy.get_param("~good_feature_distance", 5)
        self.add_feature_distance = rospy.get_param("~add_feature_distance", 10)
        self.flip_image = rospy.get_param("~flip_image", False)
        self.feature_type = rospy.get_param("~feature_type", 0) # 0 = Good Features to Track, 1 = SURF
        self.expand_roi_init = rospy.get_param("~expand_roi", 1.02)
        self.expand_roi = self.expand_roi_init
        
        self.camera_frame_id = "kinect_depth_optical_frame"
        
        self.cog_x = self.cog_y = 0
        self.cog_z = -1
            
        self.detect_box = FacesRegistry()
        self.track_box = None
        self.features = []
        
        self.grey = None
        self.pyramid = None
        self.small_image = None
        
        """ Set up the face detection parameters """
        self.cascade_frontal_alt = rospy.get_param("~cascade_frontal_alt", "")
        self.cascade_frontal_alt2 = rospy.get_param("~cascade_frontal_alt2", "")
        self.cascade_profile = rospy.get_param("~cascade_profile", "")
        
        self.cascade_frontal_alt = cv.Load(self.cascade_frontal_alt)
        self.cascade_frontal_alt2 = cv.Load(self.cascade_frontal_alt2)
        self.cascade_profile = cv.Load(self.cascade_profile)

        self.min_size = (20, 20)
        self.image_scale = 2
        self.haar_scale = 1.5
        self.min_neighbors = 1
        self.haar_flags = cv.CV_HAAR_DO_CANNY_PRUNING
        
        self.grey = None
        self.pyramid = None
        
        """ Set the Good Features to Track and Lucas-Kanade parameters """
        self.night_mode = False       
        self.quality = 0.01
        self.win_size = 10
        self.max_count = 200
        self.block_size = 3
        self.use_harris = False
        self.flags = 0
        
        self.frame_count = 0
        
        """ Set the SURF parameters """
        self.surf_hessian_quality = rospy.get_param("~surf_hessian_quality", 100)
        
        """ A service to handle 'keystroke' commands sent from other nodes """
        self.key_command = None
        rospy.Service('~key_command', KeyCommand, self.key_command_callback)
        
        """ Wait until the image topics are ready before starting """
        rospy.wait_for_message(self.input_rgb_image, Image)
        
        if self.use_depth_for_detection or self.use_depth_for_tracking:
            rospy.wait_for_message(self.input_depth_image, Image)
        
    def process_image(self, cv_image):
        #self.frame_count = self.frame_count + 1
        """ If parameter use_haar_only is True, use only the OpenCV Haar detector to track the face """
        #TODO Currently detect new faces if no faces found. Need to detcet few times epr second
        if (self.use_haar_only or not self.detect_box.faces) and self.auto_face_tracking:
            self.detect_face(cv_image)

        """ Otherwise, track the face using Good Features to Track and Lucas-Kanade Optical Flow """
        if not self.use_haar_only:
            for f in self.detect_box.faces.keys():
                if not self.detect_box.faces[f].track_box or not self.is_rect_nonzero(self.detect_box.faces[f].track_box):
                    self.detect_box.faces[f].features = []
                    self.detect_box.faces[f].update_box(self.detect_box.faces[f].face_box())
                track_box = self.track_lk(cv_image, self.detect_box.faces[f])
                rospy.loginfo("track %s", self.detect_box.faces[f].get_box())
                if len(track_box) != 3:
                    self.detect_box.faces[f].update_box(track_box)
                else:
                    self.detect_box.faces[f].update_box_elipse(track_box)

                """ Prune features that are too far from the main cluster """
                if len(self.detect_box.faces[f].features) > 0:
                    # Consider to move face class
                    ((mean_x, mean_y, mean_z), mse_xy, mse_z, score) = self.prune_features(min_features = self.detect_box.faces[f].abs_min_features, outlier_threshold = self.std_err_xy, mse_threshold=self.max_mse,face =self.detect_box.faces[f])

                    if score == -1:
                        del self.detect_box.faces[f]
                        return cv_image


                """ Add features if the number is getting too low """
                if len(self.detect_box.faces[f].features) < self.detect_box.faces[f].min_features:
                    self.detect_box.faces[f].expand_roi = self.expand_roi_init * self.detect_box.faces[f].expand_roi
                    self.add_features(cv_image,self.detect_box.faces[f])
                else:
                    self.detect_box.faces[f].expand_roi = self.expand_roi_init

        return cv_image
    
    def detect_face(self, cv_image):
        if self.grey is None:
            """ Allocate temporary images """      
            self.grey = cv.CreateImage(self.image_size, 8, 1)
            
        if self.small_image is None:
            self.small_image = cv.CreateImage((cv.Round(self.image_size[0] / self.image_scale),
                       cv.Round(self.image_size[1] / self.image_scale)), 8, 1)
    
        """ Convert color input image to grayscale """
        cv.CvtColor(cv_image, self.grey, cv.CV_BGR2GRAY)
        
        """ Equalize the histogram to reduce lighting effects. """
        cv.EqualizeHist(self.grey, self.grey)
    
        """ Scale input image for faster processing """
        cv.Resize(self.grey, self.small_image, cv.CV_INTER_LINEAR)
    
        """ First check one of the frontal templates """
        if self.cascade_frontal_alt:
            faces = cv.HaarDetectObjects(self.small_image, self.cascade_frontal_alt, cv.CreateMemStorage(0),
                                          self.haar_scale, self.min_neighbors, self.haar_flags, self.min_size)
                                         
        """ If that fails, check the profile template """
        if not faces:
            if self.cascade_profile:
                faces = cv.HaarDetectObjects(self.small_image, self.cascade_profile, cv.CreateMemStorage(0),
                                             self.haar_scale, self.min_neighbors, self.haar_flags, self.min_size)

            if not faces:
                """ If that fails, check a different frontal profile """
                if self.cascade_frontal_alt2:
                    faces = cv.HaarDetectObjects(self.small_image, self.cascade_frontal_alt2, cv.CreateMemStorage(0),
                                         self.haar_scale, self.min_neighbors, self.haar_flags, self.min_size)
            
        if not faces:
            #TODO move somewhere else as detect faces will be called while faces are tracked
            # if self.show_text:
            #     hscale = 0.4 * self.image_size[0] / 160. + 0.1
            #     vscale = 0.4 * self.image_size[1] / 120. + 0.1
            #     text_font = cv.InitFont(cv.CV_FONT_VECTOR0, hscale, vscale, 0, 1, 8)
            #     cv.PutText(self.marker_image, "LOST FACE!", (50, int(self.image_size[1] * 0.9)), text_font, cv.RGB(255, 255, 0))
            return None
        fs = []
        for ((x, y, w, h), n) in faces:
            """ The input to cv.HaarDetectObjects was resized, so scale the 
                bounding box of each face and convert it to two CvPoints """
            pt1 = (int(x * self.image_scale), int(y * self.image_scale))
            pt2 = (int((x + w) * self.image_scale), int((y + h) * self.image_scale))
            fs.append((pt1,pt2))
        rospy.loginfo("faces: %s", faces)
        self.detect_box.addFaces(fs)
        rospy.loginfo("faces2: %s", self.detect_box.faces)
        return self.detect_box.faces

    def track_lk(self, cv_image, face):
        feature_box = None
        
        """ Initialize intermediate images if necessary """
        if not self.pyramid:
            self.grey = cv.CreateImage(cv.GetSize (cv_image), 8, 1)
            self.prev_grey = cv.CreateImage(cv.GetSize (cv_image), 8, 1)
            self.pyramid = cv.CreateImage(cv.GetSize (cv_image), 8, 1)
            self.prev_pyramid = cv.CreateImage(cv.GetSize (cv_image), 8, 1)
            face.features = []
            
        """ Create a grey version of the image """
        cv.CvtColor(cv_image, self.grey, cv.CV_BGR2GRAY)
        
        """ Equalize the histogram to reduce lighting effects """
        cv.EqualizeHist(self.grey, self.grey)
            
        if face.track_box and face.features != []:
            """ We have feature points, so track and display them """
            """ Calculate the optical flow """
            face.features, status, track_error = cv.CalcOpticalFlowPyrLK(
                self.prev_grey, self.grey, self.prev_pyramid, self.pyramid,
                face.features,
                (self.win_size, self.win_size), 3,
                (cv.CV_TERMCRIT_ITER|cv.CV_TERMCRIT_EPS, 20, 0.01),
                self.flags)

            """ Keep only high status points """
            face.features = [ p for (st,p) in zip(status, face.features) if st]
                                    
        elif face.track_box and self.is_rect_nonzero(face.track_box):
            """ Get the initial features to track """
                    
            """ Create a mask image to be used to select the tracked points """
            mask = cv.CreateImage(cv.GetSize(cv_image), 8, 1) 
            
            """ Begin with all black pixels """
            cv.Zero(mask)

            """ Get the coordinates and dimensions of the track box """
            try:
                x,y,w,h = face.track_box
            except:
                return None
            
            if self.auto_face_tracking:
#                """ For faces, the detect box tends to extend beyond the actual object so shrink it slightly """
#                x = int(0.97 * x)
#                y = int(0.97 * y)
#                w = int(1 * w)
#                h = int(1 * h)
                
                """ Get the center of the track box (type CvRect) so we can create the
                    equivalent CvBox2D (rotated rectangle) required by EllipseBox below. """
                center_x = int(x + w / 2)
                center_y = int(y + h / 2)
                roi_box = ((center_x, center_y), (w, h), 0)
                
                """ Create a filled white ellipse within the track_box to define the ROI. """
                cv.EllipseBox(mask, roi_box, cv.CV_RGB(255,255, 255), cv.CV_FILLED)      
            else:
                """ For manually selected regions, just use a rectangle """
                pt1 = (x, y)
                pt2 = (x + w, y + h)
                cv.Rectangle(mask, pt1, pt2, cv.CV_RGB(255,255, 255), cv.CV_FILLED)
            
            """ Create the temporary scratchpad images """
            eig = cv.CreateImage (cv.GetSize(self.grey), 32, 1)
            temp = cv.CreateImage (cv.GetSize(self.grey), 32, 1)

            if self.feature_type == 0:
                """ Find keypoints to track using Good Features to Track """
                face.features = cv.GoodFeaturesToTrack(self.grey, eig, temp, self.max_count,
                    self.quality, self.good_feature_distance, mask=mask, blockSize=self.block_size, useHarris=self.use_harris, k=0.04)
            
            elif self.feature_type == 1:
                """ Get the new features using SURF """
                (surf_features, descriptors) = cv.ExtractSURF(self.grey, mask, cv.CreateMemStorage(0), (0, self.surf_hessian_quality, 3, 1))
                for feature in surf_features:
                    face.features.append(feature[0])
            #
            if self.auto_min_features:
                """ Since the detect box is larger than the actual face or desired patch, shrink the number of features by 10% """
                face.min_features = int(len(face.features) * 0.9)
                face.abs_min_features = int(0.5 * face.min_features)
        
        """ Swapping the images """
        self.prev_grey, self.grey = self.grey, self.prev_grey
        self.prev_pyramid, self.pyramid = self.pyramid, self.prev_pyramid
        
        """ If we have some features... """
        if len(face.features) > 0:
            """ The FitEllipse2 function below requires us to convert the feature array
                into a CvMat matrix """
            try:
                self.feature_matrix = cv.CreateMat(1, len(face.features), cv.CV_32SC2)
            except:
                pass
                        
            """ Draw the points as green circles and add them to the features matrix """
            i = 0
            for the_point in face.features:
                if self.show_features:
                    cv.Circle(self.marker_image, (int(the_point[0]), int(the_point[1])), 2, (0, 255, 0, 0), cv.CV_FILLED, 8, 0)
                try:
                    cv.Set2D(self.feature_matrix, 0, i, (int(the_point[0]), int(the_point[1])))
                except:
                    pass
                i = i + 1
    
            """ Draw the best fit ellipse around the feature points """
            if len(face.features) > 6:
                feature_box = cv.FitEllipse2(self.feature_matrix)
            else:
                feature_box = None
            
            """ Publish the ROI for the tracked object """
            try:
                (roi_center, roi_size, roi_angle) = feature_box
            except:
                rospy.loginfo("Patch box has shrunk to zeros...")
                feature_box = None
    
            if feature_box and not self.drag_start and self.is_rect_nonzero(face.track_box):
                self.ROI = RegionOfInterest()
                self.ROI.x_offset = min(self.image_size[0], max(0, int(roi_center[0] - roi_size[0] / 2)))
                self.ROI.y_offset = min(self.image_size[1], max(0, int(roi_center[1] - roi_size[1] / 2)))
                self.ROI.width = min(self.image_size[0], int(roi_size[0]))
                self.ROI.height = min(self.image_size[1], int(roi_size[1]))
                
            self.pubROI.publish(self.ROI)

            
        if feature_box is not None and len(face.features) > 0:
            return feature_box
        else:
            return None
        
    def add_features(self, cv_image, face):
        """ Look for any new features around the current feature cloud """
        
        """ Create the ROI mask"""
        roi = cv.CreateImage(cv.GetSize(cv_image), 8, 1) 
        
        """ Begin with all black pixels """
        cv.Zero(roi)

        """ Get the coordinates and dimensions of the current track box """
        try:
            ((x,y), (w,h), a) = face.track_box
        except:
            rospy.loginfo("Track box has shrunk to zero...")
            return
        
        """ Expand the track box to look for new features """
        w = int(face.expand_roi * w)
        h = int(face.expand_roi * h)
        
        roi_box = ((x,y), (w,h), a)
        
        """ Create a filled white ellipse within the track_box to define the ROI. """
        cv.EllipseBox(roi, roi_box, cv.CV_RGB(255,255, 255), cv.CV_FILLED)
        
        """ Create the temporary scratchpad images """
        eig = cv.CreateImage (cv.GetSize(self.grey), 32, 1)
        temp = cv.CreateImage (cv.GetSize(self.grey), 32, 1)
        
        if self.feature_type == 0:
            """ Get the new features using Good Features to Track """
            features = cv.GoodFeaturesToTrack(self.grey, eig, temp, self.max_count,
            self.quality, self.good_feature_distance, mask=roi, blockSize=3, useHarris=0, k=0.04)
        
        elif self.feature_type == 1:
            """ Get the new features using SURF """
            features = []
            (surf_features, descriptors) = cv.ExtractSURF(self.grey, roi, cv.CreateMemStorage(0), (0, self.surf_hessian_quality, 3, 1))
            for feature in surf_features:
                features.append(feature[0])
                
        """ Append new features to the current list if they are not too far from the current cluster """
        for new_feature in features:
            try:
                distance = self.distance_to_cluster(new_feature, face.features)
                if distance > self.add_feature_distance:
                    face.features.append(new_feature)
            except:
                pass
                
        """ Remove duplicate features """
        face.features = list(set(face.features))

    def distance_to_cluster(self, test_point, cluster):
        min_distance = 10000
        for point in cluster:
            if point == test_point:
                continue
            """ Use L1 distance since it is faster than L2 """
            distance = abs(test_point[0] - point[0]) + abs(test_point[1] - point[1])
            if distance < min_distance:
                min_distance = distance
        return min_distance
    
    def get_cluster_centroid(self):
        """ compute the 3D centroid (COG) of the current cluster """
        n_xy = n_z = 0
        sum_x = sum_y = sum_z = 0
        
        (cols, rows) = cv.GetSize(self.depth_image)
        
        for point in self.features:
            sum_x = sum_x + point[0]
            sum_y = sum_y + point[1]
            n_xy += 1
            
            try:
                z = cv.Get2D(self.depth_image, min(rows - 1, int(point[1])), min(cols - 1, int(point[0])))
            except cv.error:
                rospy.loginfo("Get2D Index Error: " + str(int(point[1])) + " x " + str(int(point[0])))
                continue

            """ Depth values can be NaN which should be ignored """
            if isnan(z[0]):
                continue
            else:
                sum_z = sum_z + z[0]
                n_z += 1
                
        #rospy.loginfo(n_z)
        
        if n_xy > 0:
            cog_x = sum_x / n_xy
            cog_y = sum_y / n_xy
            
        """ The Kinect returns NaN depth values when closer than about 0.5 meters.  If the target is closer than 0.5 meters
            then use 0.5 meters as a fudge """
        if n_z > 0:
           cog_z = sum_z / n_z
        else:
            cog_z = 0.5
        # Convert the cog_x and cog_y pixel values to meters using the fact that the Kinect's FOV is about 57 degrees or 1 radian.
        cog_x = cog_z * self.fov_width * (cog_x - self.image_size[0] / 2.0) / float(self.image_size[0])
        cog_y = cog_z * self.fov_height * (cog_y - self.image_size[1] / 2.0) / float(self.image_size[1])
                        
        return (cog_x, cog_y, cog_z)    
    
    def prune_features(self, min_features, outlier_threshold, mse_threshold, face):
        sum_x = 0
        sum_y = 0
        sum_z = 0
        sse = 0
        features_xy = face.features
        features_z = face.features
        n_xy = len(face.features)
        n_z = 0
        mean_z = mse_z = -1
        
        """ If there are no features left to track, start over """
        if n_xy == 0:
            return ((0, 0, 0), 0, 0, -1)
        
        """ Compute the COG (center of gravity) of the cluster """
        for point in face.features:
            sum_x = sum_x + point[0]
            sum_y = sum_y + point[1]
        
        mean_x = sum_x / n_xy
        mean_y = sum_y / n_xy

        """ Compute the x-y MSE (mean squared error) of the cluster in the camera plane """
        for point in face.features:
            sse = sse + (point[0] - mean_x) * (point[0] - mean_x) + (point[1] - mean_y) * (point[1] - mean_y)
            #sse = sse + abs((point[0] - mean_x)) + abs((point[1] - mean_y))
        
        """ Get the average over the number of feature points """
        mse_xy = sse / n_xy
        
        """ The MSE must be > 0 for any sensible feature cluster """
        if mse_xy == 0 or mse_xy > mse_threshold:
            return ((0, 0, 0), 0, 0, -1)
        
        """ Throw away the outliers based on the x-y variance """
        max_err = 0
        for point in face.features:
            std_err = ((point[0] - mean_x) * (point[0] - mean_x) + (point[1] - mean_y) * (point[1] - mean_y)) / mse_xy
            if std_err > max_err:
                max_err = std_err
            if std_err > outlier_threshold:
                features_xy.remove(point)
                try:
                	features_z.remove(point)
                	n_z = n_z - 1
                except:
                	pass
                
                n_xy = n_xy - 1

        face.features = features_xy
               
        """ Consider a cluster bad if we have fewer than abs_min_features left """
        if len(face.features) < face.abs_min_features:
            score = -1
        else:
            score = 1
                            
        return ((mean_x, mean_y, mean_z), mse_xy, mse_z, score)
    
    def key_command_callback(self, req):
        self.key_command = req.command
        return KeyCommandResponse()

    def set_roi_callback(self, req):
        self.keypoints = []
        self.track_box = None
        self.detect_box = (req.roi.x_offset, req.roi.y_offset, req.roi.width, req.roi.height)
        return SetROIResponse()

def main(args):
    """ Display a help message if appropriate """
    help_message =  "Hot keys: \n" \
          "\tq - quit the program\n" \
          "\tc - delete current features\n" \
          "\tt - toggle text captions on/off\n" \
          "\tf - toggle display of features on/off\n" \
          "\tn - toggle \"night\" mode on/off\n" \
          "\ta - toggle auto face tracking on/off\n"

    print help_message
    
    """ Fire up the Face Tracker node """
    PT = PatchTracker("pi_face_tracker")

    try:
      rospy.spin()
    except KeyboardInterrupt:
      print "Shutting down face tracker node."
      cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)