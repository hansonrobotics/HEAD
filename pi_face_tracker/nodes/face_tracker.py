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
from ros2opencv import ROS2OpenCV
import rospy
import cv
import sys
import os
from sensor_msgs.msg import RegionOfInterest, Image
from math import sqrt, isnan

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
            
        self.detect_box = None
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
        self.flags = 0
        self.frame_count = 0
        
        """ Set the SURF parameters """
        self.surf_hessian_quality = rospy.get_param("~surf_hessian_quality", 100)
                
        """ Wait until the image topics are ready before starting """
        rospy.wait_for_message(self.input_rgb_image, Image)
        
        if self.use_depth_for_detection or self.use_depth_for_tracking:
            rospy.wait_for_message(self.input_depth_image, Image)
        
    def process_image(self, cv_image):
        #self.frame_count = self.frame_count + 1
        """ If parameter use_haar_only is True, use only the OpenCV Haar detector to track the face """
        if (self.use_haar_only or not self.detect_box) and self.auto_face_tracking:
            self.detect_box = self.detect_face(cv_image)

        """ Otherwise, track the face using Good Features to Track and Lucas-Kanade Optical Flow """
        if not self.use_haar_only:
            if self.detect_box:
                if not self.track_box or not self.is_rect_nonzero(self.track_box):
                    self.features = []
                    self.track_box = self.detect_box
                self.track_box = self.track_lk(cv_image)
                
                """ Prune features that are too far from the main cluster """
                if len(self.features) > 0:
                    ((mean_x, mean_y, mean_z), mse_xy, mse_z, score) = self.prune_features(min_features = self.abs_min_features, outlier_threshold = self.std_err_xy, mse_threshold=self.max_mse)
                    
                    if score == -1:
                        self.detect_box = None
                        self.track_box = None
                        return cv_image
                
                """ Add features if the number is getting too low """
                if len(self.features) < self.min_features:
                    self.expand_roi = self.expand_roi_init * self.expand_roi
                    self.add_features(cv_image)
                else:
                    self.expand_roi = self.expand_roi_init      
            else:
                self.features = []
                self.track_box = None
        
        # If using depth info, get the cluster centroid
        if len(self.features) > 0 and (self.use_depth_for_detection or self.use_depth_for_tracking):
            (self.cog_x, self.cog_y, self.cog_z) = self.get_cluster_centroid()
        
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
            if self.show_text:
                text_font = cv.InitFont(cv.CV_FONT_VECTOR0, 3, 2, 0, 3)
                cv.PutText(self.marker_image, "LOST FACE!", (50, int(self.image_size[1] * 0.9)), text_font, cv.RGB(255, 255, 0))
            return None
                
        for ((x, y, w, h), n) in faces:
            """ The input to cv.HaarDetectObjects was resized, so scale the 
                bounding box of each face and convert it to two CvPoints """
            pt1 = (int(x * self.image_scale), int(y * self.image_scale))
            pt2 = (int((x + w) * self.image_scale), int((y + h) * self.image_scale))
            face_width = pt2[0] - pt1[0]
            face_height = pt2[1] - pt1[1]

            if self.use_depth_for_detection:
                """ Get the average distance over the face box """
                ave_face_distance = 0
                i = 0
                for x in range(pt1[0], pt2[0]):
                    for y in range(pt1[1], pt2[1]):
                        try:
                            face_distance = cv.Get2D(self.depth_image, y, x)
                            z = face_distance[0]
                        except:
                            continue
                        if isnan(z):
                            continue
                        else:
                            ave_face_distance += z
                            i = i + 1

                """ If we are too close to the Kinect, we will get NaN for distances so just accept the detection. """
                if i == 0:
                    face_size = 0
                
                else:
                    """ Compute the size of the face in meters (average of width and height)
                        The Kinect's FOV is about 57 degrees wide which is, coincidentally about 1 radian.
                    """
                    ave_face_distance = ave_face_distance / float(i)
                    arc = (self.fov_width * float(face_width) / float(self.image_size[0]) + self.fov_height * float(face_height) / float(self.image_size[1])) / 2.0
                    face_size = ave_face_distance * arc
                
                if face_size > self.max_face_size:
                    continue
                
            face_box = (pt1[0], pt1[1], face_width, face_height)

            """ Break out of the loop after the first face """
            return face_box

    def track_lk(self, cv_image):
        feature_box = None
        
        """ Initialize intermediate images if necessary """
        if not self.pyramid:
            self.grey = cv.CreateImage(cv.GetSize (cv_image), 8, 1)
            self.prev_grey = cv.CreateImage(cv.GetSize (cv_image), 8, 1)
            self.pyramid = cv.CreateImage(cv.GetSize (cv_image), 8, 1)
            self.prev_pyramid = cv.CreateImage(cv.GetSize (cv_image), 8, 1)
            self.features = []
            
        """ Create a grey version of the image """
        cv.CvtColor(cv_image, self.grey, cv.CV_BGR2GRAY)
        
        """ Equalize the histogram to reduce lighting effects """
        cv.EqualizeHist(self.grey, self.grey)
        #cv.Smooth(self.grey, self.grey, param1=13)
            
        if self.track_box and self.features != []:
            """ We have feature points, so track and display them """

            """ Calculate the optical flow """
            self.features, status, track_error = cv.CalcOpticalFlowPyrLK(
                self.prev_grey, self.grey, self.prev_pyramid, self.pyramid,
                self.features,
                (self.win_size, self.win_size), 3,
                (cv.CV_TERMCRIT_ITER|cv.CV_TERMCRIT_EPS, 20, 0.01),
                self.flags)

            """ Keep only high status points """
            self.features = [ p for (st,p) in zip(status, self.features) if st]        
                                    
        elif self.track_box and self.is_rect_nonzero(self.track_box):
            """ Get the initial features to track """
                    
            """ Create a mask image to be used to select the tracked points """
            mask = cv.CreateImage(cv.GetSize(cv_image), 8, 1) 
            
            """ Begin with all black pixels """
            cv.Zero(mask)

            """ Get the coordinates and dimensions of the track box """
            try:
                x,y,w,h = self.track_box
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
                self.features = cv.GoodFeaturesToTrack(self.grey, eig, temp, self.max_count,
                    self.quality, self.good_feature_distance, mask=mask, blockSize=3, useHarris=0, k=0.04)
            
            elif self.feature_type == 1:
                """ Get the new features using SURF """
                (surf_features, descriptors) = cv.ExtractSURF(self.grey, mask, cv.CreateMemStorage(0), (0, self.surf_hessian_quality, 3, 1))
                for feature in surf_features:
                    self.features.append(feature[0])
            
            if self.auto_min_features:
                """ Since the detect box is larger than the actual face or desired patch, shrink the number a features by 10% """
                self.min_features = int(len(self.features) * 0.9)
                self.abs_min_features = int(0.5 * self.min_features)
        
        """ Swapping the images """
        self.prev_grey, self.grey = self.grey, self.prev_grey
        self.prev_pyramid, self.pyramid = self.pyramid, self.prev_pyramid
        
        """ If we have some features... """
        if len(self.features) > 0:
            """ The FitEllipse2 function below requires us to convert the feature array
                into a CvMat matrix """
            try:
                self.feature_matrix = cv.CreateMat(1, len(self.features), cv.CV_32SC2)
            except:
                pass
                        
            """ Draw the points as green circles and add them to the features matrix """
            i = 0
            for the_point in self.features:
                if self.show_features:
                    cv.Circle(self.marker_image, (int(the_point[0]), int(the_point[1])), 3, (0, 255, 0, 0), cv.CV_FILLED, 8, 0)
                try:
                    cv.Set2D(self.feature_matrix, 0, i, (int(the_point[0]), int(the_point[1])))
                except:
                    pass
                i = i + 1
    
            """ Draw the best fit ellipse around the feature points """
            if len(self.features) > 6:
                feature_box = cv.FitEllipse2(self.feature_matrix)
            else:
                feature_box = None
            
            """ Publish the ROI for the tracked object """
            try:
                (roi_center, roi_size, roi_angle) = feature_box
            except:
                rospy.loginfo("Patch box has shrunk to zero...")
                feature_box = None
    
            if feature_box and not self.drag_start and self.is_rect_nonzero(self.track_box):
                self.ROI = RegionOfInterest()
                self.ROI.x_offset = min(self.image_size[0], max(0, int(roi_center[0] - roi_size[0] / 2)))
                self.ROI.y_offset = min(self.image_size[1], max(0, int(roi_center[1] - roi_size[1] / 2)))
                self.ROI.width = min(self.image_size[0], int(roi_size[0]))
                self.ROI.height = min(self.image_size[1], int(roi_size[1]))
                
            self.pubROI.publish(self.ROI)
            
            """ If using depth info Publish the centroid of the tracked cluster as a PointStamped message """
            if self.use_depth_for_detection or self.use_depth_for_tracking:
                if feature_box is not None and not self.drag_start and self.is_rect_nonzero(self.track_box):
                    self.cluster3d.header.frame_id = self.camera_frame_id
                    self.cluster3d.header.stamp = rospy.Time()
                    self.cluster3d.point.x = self.cog_x
                    self.cluster3d.point.y = self.cog_y
                    self.cluster3d.point.z = self.cog_z
                    self.pub_cluster3d.publish(self.cluster3d)
            
        if feature_box is not None and len(self.features) > 0:
            return feature_box
        else:
            return None
        
    def add_features(self, cv_image):
        """ Look for any new features around the current feature cloud """
        
        """ Create the ROI mask"""
        roi = cv.CreateImage(cv.GetSize(cv_image), 8, 1) 
        
        """ Begin with all black pixels """
        cv.Zero(roi)
        
        """ Get the coordinates and dimensions of the current track box """
        try:
            ((x,y), (w,h), a) = self.track_box
        except:
            rospy.loginfo("Track box has shrunk to zero...")
            return
        
        """ Expand the track box to look for new features """
        w = int(self.expand_roi * w)
        h = int(self.expand_roi * h)
        
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
                distance = self.distance_to_cluster(new_feature, self.features)
                if distance > self.add_feature_distance:
                    self.features.append(new_feature)
            except:
                pass
                
        """ Remove duplicate features """
        self.features = list(set(self.features))

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
    
    def prune_features(self, min_features, outlier_threshold, mse_threshold):
        sum_x = 0
        sum_y = 0
        sum_z = 0
        sse = 0
        features_xy = self.features
        features_z = self.features
        n_xy = len(self.features)
        n_z = 0
        mean_z = mse_z = -1
        
        if self.use_depth_for_tracking:
            if not self.depth_image:
                return ((0, 0, 0), 0, 0, -1)
            else:
                (cols, rows) = cv.GetSize(self.depth_image)
        
        """ If there are no features left to track, start over """
        if n_xy == 0:
            return ((0, 0, 0), 0, 0, -1)
        
        """ Compute the COG (center of gravity) of the cluster """
        for point in self.features:
            sum_x = sum_x + point[0]
            sum_y = sum_y + point[1]
        
        mean_x = sum_x / n_xy
        mean_y = sum_y / n_xy

        """ Compute the x-y MSE (mean squared error) of the cluster in the camera plane """
        for point in self.features:
            sse = sse + (point[0] - mean_x) * (point[0] - mean_x) + (point[1] - mean_y) * (point[1] - mean_y)
            #sse = sse + abs((point[0] - mean_x)) + abs((point[1] - mean_y))
        
        """ Get the average over the number of feature points """
        mse_xy = sse / n_xy
        
        """ The MSE must be > 0 for any sensible feature cluster """
        if mse_xy == 0 or mse_xy > mse_threshold:
            return ((0, 0, 0), 0, 0, -1)
        
        """ Throw away the outliers based on the x-y variance """
        max_err = 0
        for point in self.features:
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
                                
        """ Now do the same for depth """
        if self.use_depth_for_tracking:
            for point in self.features:
                try:
                    z = cv.Get2D(self.depth_image, min(rows - 1, int(point[1])), min(cols - 1, int(point[0])))
                except:
                    continue

                """ Depth values can be NaN which should be ignored """
                if isnan(z[0]):
                    continue
                else:
                    sum_z = sum_z + z[0]
                    n_z += 1
            
            if n_z != 0:        
                mean_z = sum_z / n_z
                
                sse = 0
                n_z = 0
                for point in self.features:
                    try:
                        z = cv.Get2D(self.depth_image, min(rows - 1, int(point[1])), min(cols - 1, int(point[0])))
                        sse = sse + (z[0] - mean_z) * (z[0] - mean_z)
                        n_z += 1
                    except:
                        continue
                
                if n_z != 0:
                    mse_z = sse / n_z
                    
                    """ Throw away the outliers based on depth using percent error rather than standard error since depth
                         values can jump dramatically at object boundaries  """
                    for point in features_z:
                        try:
                            z = cv.Get2D(self.depth_image, min(rows - 1, int(point[1])), min(cols - 1, int(point[0])))
                        except:
                            continue
                        try:
                            pct_err = abs(z[0] - mean_z) / mean_z
                            if pct_err > self.pct_err_z:
                                features_xy.remove(point)
                        except:
                            pass
        
        self.features = features_xy
               
        """ Consider a cluster bad if we have fewer than abs_min_features left """
        if len(self.features) < self.abs_min_features:
            score = -1
        else:
            score = 1
                            
        return ((mean_x, mean_y, mean_z), mse_xy, mse_z, score)     

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