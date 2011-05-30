#!/usr/bin/env python

""" face_tracker.py - Version 1.0 2011-04-28

    Track a face using the OpenCV Haar detector to initially locate the face, then OpenCV's
    Good Features to Track and Lucas-Kanade Optical Flow to track face features over 
    subsequent frames.
    
    Can also be used to track arbitrarily selected patches by setting the parameter
    do_face_tracking to False and using the mouse to select the desired region.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2011 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
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
from sensor_msgs.msg import RegionOfInterest
from math import sqrt, isnan

class PatchTracker(ROS2OpenCV):
    def __init__(self, node_name):
        ROS2OpenCV.__init__(self, node_name)
        
        self.node_name = node_name
        
        """ Do automatic face tracking? """
        self.do_face_tracking = rospy.get_param("~do_face_tracking", True)

        """ For face tracking, use only the OpenCV face detector? """
        self.detect_face_only = rospy.get_param("~detect_face_only", False)
        
        """ Should we use depth information for detection? """
        self.use_depth_for_detection = rospy.get_param("~use_depth_for_detection", False)
        
        self.fov_width = rospy.get_param("~fov_width", 1.0)
        self.fov_height = rospy.get_param("~fov_height", 1.0)
        
        """ What is the maximum size (in meters) we will accept for a face detection? """
        self.max_face_size = rospy.get_param("~max_face_size", 0.28)

        """ Should we use depth information for tracking? """
        self.use_depth_for_tracking = rospy.get_param("~use_depth_for_tracking", False)
        
        """ What is the minimum number of feature we will accept before expanding the track window?
            Use 50 for a 640x480 image, or 25 for a 320x240 image """
        self.min_features = rospy.get_param("~min_features", 50)
        
        """ What is the smallest number of features we will accept before returning to the detector
            to get a fresh patch? """
        self.abs_min_features = rospy.get_param("~abs_min_features", 6)    
        
        """ How much should we expand the track box when the number of features falls below threshold? """
        self.expand_scale = 1.5  
            
        self.detect_box = None
        self.track_box = None
        self.features = []
        
        self.grey = None
        self.pyramid = None
        
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
        self.min_distance = 5
        self.win_size = 10
        self.max_count = 200
        self.flags = 0
        
    def process_image(self, cv_image):
        """ If parameter detect_face_only is True, use only the OpenCV Haar detector to track the face """
        if (self.detect_face_only or not self.detect_box) and self.do_face_tracking:
            self.detect_box = self.detect_face(cv_image)
        
        """ Otherwise, track the face using Good Features to Track and Lucas-Kanade Optical Flow """
        if not self.detect_face_only:
            if self.detect_box:
                if not self.track_box:
                    self.features = []
                    self.track_box = self.detect_box
                self.track_box = self.track_lk(cv_image)
                
                if len(self.features) < self.min_features:
                    self.expand_scale = 1.1 * self.expand_scale
                    self.add_features(cv_image, min_distance=10)
                else:
                    self.expand_scale = 1.1
            else:
                self.features = []
                self.track_box = None
        
        return cv_image
    
    def detect_face(self, cv_image):        
        if self.grey is None:
            """ Allocate temporary images """      
            self.grey = cv.CreateImage(self.image_size, 8, 1)
            self.small_image = cv.CreateImage((cv.Round(self.image_size[0] / self.image_scale),
                       cv.Round(self.image_size[1] / self.image_scale)), 8, 1)
    
        """ Convert color input image to grayscale """
        cv.CvtColor(cv_image, self.grey, cv.CV_BGR2GRAY)
        
        """ Equalize the histogram to reduce lighting effects. """
        cv.EqualizeHist(self.grey, self.grey)
    
        """ Scale input image for faster processing """
        cv.Resize(self.grey, self.small_image, cv.CV_INTER_LINEAR)
    
        """ First check the profile template """
        if self.cascade_profile:
            faces = cv.HaarDetectObjects(self.small_image, self.cascade_profile, cv.CreateMemStorage(0),
                                         self.haar_scale, self.min_neighbors, self.haar_flags, self.min_size)
            if not faces:
                """ If the alt frontal template fails, check the profile template """
                if self.cascade_frontal_alt:
                    faces = cv.HaarDetectObjects(self.small_image, self.cascade_frontal_alt, cv.CreateMemStorage(0),
                                         self.haar_scale, self.min_neighbors, self.haar_flags, self.min_size)
            if not faces:
                """ If the alt2 frontal template fails, check the profile template """
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
                """ Get the distance to the middle of the face box """
                face_distance = cv.Get2D(self.depth_image, int(pt1[1] + face_height / 2), int(pt1[0] + face_width / 2))
                face_distance = face_distance[0]
                
                """ If we are too close to the Kinect, we will get NaN for distance so just accept the detection. """
                if isnan(face_distance):
                    face_size = 0
                
                else:
                    """ Compute the size of the face in meters (average of width and height)
                        The Kinect's FOV is about 57 degrees wide which is, coincidentally about 1 radian.
                    """
                    arc = (self.fov_width * float(face_width) / float(self.image_size[0]) + self.fov_height * float(face_height) / float(self.image_size[1])) / 2.0
                    face_size = face_distance * arc
                
                if face_size > self.max_face_size:
                    continue
                
            face_box = (pt1[0], pt1[1] , face_width, face_height)

            """ Break out of the loop after the first face """
            return face_box
        
    def track_lk(self, cv_image):
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
                    
            """ Create the ROI mask"""
            roi = cv.CreateImage(cv.GetSize(cv_image), 8, 1) 
            
            """ Begin with all black pixels """
            cv.Zero(roi)

            """ Get the coordinates and dimensions of the track box """
            try:
                x,y,w,h = self.track_box
            except:
                return None
            
            """ The detect box tends to extend beyond the actual object so shrink it slightly """
            x = int(0.97 * x)
            y = int(0.97 * y)
            w = int(1 * w)
            h = int(1 * h)
            
            """ Get the center of the track box (type CvRect) so we can create the
                equivalent CvBox2D (rotated rectangle) required by EllipseBox below. """
            center_x = int(x + w / 2)
            center_y = int(y + h / 2)
            roi_box = ((center_x, center_y), (w, h), 0)
            
            """ Create a white ellipse within the track_box to define the ROI.
                A thickness of -1 causes ellipse to be filled with chosen color, in this case white. """
            cv.EllipseBox(roi, roi_box, cv.CV_RGB(255,255, 255), thickness=-1)
            
            """ Create the temporary scratchpad images """
            eig = cv.CreateImage (cv.GetSize(self.grey), 32, 1)
            temp = cv.CreateImage (cv.GetSize(self.grey), 32, 1)

            """ Find points to track using Good Features to Track """
            self.features = cv.GoodFeaturesToTrack(self.grey, eig, temp, self.max_count,
                self.quality, self.min_distance, mask=roi, blockSize=3, useHarris=0, k=0.04)
        
        
        """ Swapping the images """
        self.prev_grey, self.grey = self.grey, self.prev_grey
        self.prev_pyramid, self.pyramid = self.pyramid, self.prev_pyramid
        
        """ If we have some features... """
        if len(self.features) > 0:
            """ Check the spread of the feature cluster """
            ((cog_x, cog_y, cog_z), mse_xy, mse_z, score) = self.prune_features(min_features = self.abs_min_features, outlier_threshold = 3.0, mse_threshold=5000)
            
            if score == -1:
                self.detect_box = None
                self.track_box = None
                return None
        
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
                    cv.Circle(self.marker_image, (int(the_point[0]), int(the_point[1])), 3, (0, 255, 0, 0), -1, 8, 0)
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
                self.ROI.x_offset = int(roi_center[0] - roi_size[0] / 2)
                self.ROI.y_offset = int(roi_center[1] - roi_size[1] / 2)
                self.ROI.width = int(roi_size[0])
                self.ROI.height = int(roi_size[1])
                
            self.pubROI.publish(self.ROI)
            
        if feature_box is not None and len(self.features) > 0:
            return feature_box
        else:
            return None
        
    def add_features(self, cv_image, min_distance):
        """ Look for any new features around the current track box (ellipse) """
        
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
        w = int(self.expand_scale * w)
        h = int(self.expand_scale * h)
        
        roi_box = ((x,y), (w,h), a)
        
        """ Create a white ellipse within the track_box to define the ROI.
        A thickness of -1 causes ellipse to be filled with chosen color,
        in this case white. """
        cv.EllipseBox(roi, roi_box, cv.CV_RGB(255,255, 255), thickness=-1)
        
        """ Create the temporary scratchpad images """
        eig = cv.CreateImage (cv.GetSize(self.grey), 32, 1)
        temp = cv.CreateImage (cv.GetSize(self.grey), 32, 1)
        
        """ Get the new features using Good Features to Track """
        features = cv.GoodFeaturesToTrack(self.grey, eig, temp, self.max_count,
        self.quality, self.min_distance, mask=roi, blockSize=3, useHarris=0, k=0.04)
                
        """ Append new features to the current list """
        i = 0
        for new_feature in features:
            if self.distance_to_cluster(new_feature, self.features) > min_distance:
                self.features.append(new_feature)
                i = i + 1
                
        """ Remove duplicate features """
        self.features = list(set(self.features))

    def distance_to_cluster(self, test_point, cluster):
        min_distance = 10000
        for point in cluster:
            distance = abs(test_point[0] - point[0])  + abs(test_point[1] - point[1])
            if distance < min_distance:
                min_distance = distance
        return min_distance
    
    def prune_features(self, min_features, outlier_threshold, mse_threshold):
        sum_x = 0
        sum_y = 0
        sum_z = 0
        sse = 0
        features_xy = self.features
        features_z = self.features
        n_xy = len(self.features)
        n_z = n_xy
        
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
            if self.use_depth_for_tracking:
                try:
                    z = cv.Get2D(self.depth_image, min(rows - 1, int(point[1])), min(cols - 1, int(point[0])))
                except:
                    rospy.loginfo("X: " + str(point[1]) + " Y: " + str(point[0]))
                    return ((0, 0, 0), 0, 0, -1)

                z = z[0]
                """ Depth values can be NaN which will cause an exception and should be dropped """
                if isnan(z):
                    features_z.remove(point)
                    n_z = n_z - 1
                else:
                    sum_z = sum_z + z
        
        mean_x = sum_x / n_xy
        mean_y = sum_y / n_xy
        
        if self.use_depth_for_tracking:
            mean_z = sum_z / n_z
        else:
            mean_z = -1
        
        """ Compute the x-y MSE (mean squared error) of the cluster in the camera plane """
        for point in self.features:
            sse = sse + (point[0] - mean_x) * (point[0] - mean_x) + (point[1] - mean_y) * (point[1] - mean_y)
        
        """ Get the average over the number of feature points """
        mse_xy = sse / n_xy
        
        """ The MSE must be > 0 for any sensible feature cluster """
        if mse_xy == 0 or mse_xy > mse_threshold:
            return ((0, 0, 0), 0, 0, -1)
        
        """ Throw away the outliers based on the x-y variance """
        for point in self.features:
            if ((point[0] - mean_x) * (point[0] - mean_x) + (point[1] - mean_y) * (point[1] - mean_y)) / mse_xy > outlier_threshold:
				features_xy.remove(point)
				try:
					features_z.remove(point)
					n_z = n_z - 1
				except:
					pass
				
				n_xy = n_xy - 1
                
        """ Now do the same for depth """
        if self.use_depth_for_tracking:
            sse = 0
            for point in features_z:
				z = cv.Get2D(self.depth_image, min(rows - 1, int(point[1])), min(cols - 1, int(point[0])))
				z = z[0]
				sse = sse + (z - mean_z) * (z - mean_z)
            
            mse_z = sse / n_z
            
            """ Throw away the outliers based on depth """
            for point in features_z:
                z = cv.Get2D(self.depth_image, min(rows - 1, int(point[1])), min(cols - 1, int(point[0])))
                z = z[0]
                try:
                    if abs(z - mean_z) / mean_z > 0.5:
                        features_xy.remove(point)
                except:
                    pass
        else:
            mse_z = -1
        
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
          "\tn - toggle the \"night\" mode on/off\n"

    print help_message
    
    """ Fire up the Face Tracker node """
    PT = PatchTracker("face_tracker")

    try:
      rospy.spin()
    except KeyboardInterrupt:
      print "Shutting down face tracker node."
      cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)