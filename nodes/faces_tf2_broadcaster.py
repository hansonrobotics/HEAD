#!/usr/bin/env python
#
# 3D tracking of faces in the visual field of a camera.
# Copyright (C) 2014 Hanson Robotics
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
# 02110-1301  USA

import rospy
import tf
from pi_face_tracker.msg import *
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header
import time
import math

broadcasters = {}

class FacesPublisher:

    def __init__(self):
        # distance allowed by two point to merge them
        self.max_distance = rospy.get_param('max_distance', 0.5)
        # angle for faces to be marked as same, to be refined in future
        #self.max_angle = rospy.get_param('max_angle', 0.085)
        rospy.Subscriber('camera/face_locations',
                     pi_face_tracker.msg.Faces,
                     self.handle_faces)
        rospy.Subscriber('eye_camera/face_locations',
                     pi_face_tracker.msg.Faces,
                     self.handle_eye_faces)
        # Last faces gotten from body camera
        self.last_faces = {}
        # Stores delta information for the faces
        self.delta = {}
        self.broadcaster =  tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        # Wait for camera and eye transforms
        try:
            self.listener.waitForTransform('camera', 'world', \
                rospy.Time(0), rospy.Duration(3.0))
        except Exception:
            rospy.logerr("No camera transforms!")
            exit(1)

        self.last_faces_time = rospy.Time.now()
        #self.buffer.lookup_transform()

    def handle_faces(self, msg):
        self.last_faces = msg.faces
        self.last_faces_time = now = rospy.Time.now()
        for i, face in enumerate(msg.faces):
            # tr= TransformStamped(header=Header(stamp=rospy.Time.now(), frame_id="camera"),
            #                  transform=Transform(translation=Vector3(face.point.x,face.point.y,face.point.z),
            #                                      rotation=Quaternion(0,0,0,1)),
            #                  child_frame_id="face_base" + str(face.id)
            #                 )
            # self.broadcaster.sendTransform(tr)    --- tf2 version

            self.broadcaster.sendTransform((face.point.x,face.point.y,face.point.z),(0,0,0,1),
                                            self.last_faces_time,"face_base" + str(face.id),"camera")
            time.sleep(0.005) # Need to investigate why we cant publish multiple messages at same time
            # Publish eye detected face
            # t = TransformStamped(header=Header(stamp=rospy.Time.now(), frame_id="face_base" + str(face.id)),
            #                      child_frame_id="Face" + str(face.id)
            #                      ) -- tf2 version
            p = (0,0,0)
            if face.id in self.delta:
                #t.transform.translation = self.delta[face.id]
                d = self.delta[face.id]
                p=(d.x,d.y,d.z)

            #self.broadcaster.sendTransform(t)  --- tf2 version
            self.broadcaster.sendTransform(p, (0, 0, 0, 1),  self.last_faces_time,
                                           "Face" + str(face.id), "face_base" + str(face.id))


    def handle_eye_faces(self, msg):
        # Clear corrections everytime frame from eyes processed
        self.delta = {}
        for i, eye_face in enumerate(msg.faces):
            # used for debug
            #self.broadcaster.sendTransform((eye_face.point.x,eye_face.point.y,eye_face.point.z),(0,0,0,1),
            #                                self.last_faces_time,"eye_face" + str(eye_face.id),"right_eye")
            ps = PointStamped()
            ps.point = eye_face.point
            ps.header=Header(stamp= rospy.Time(0), frame_id="right_eye")
            for j, face in enumerate(self.last_faces):
                try:

                    pst = self.listener.transformPoint("face_base"+str(face.id),ps)
                    if self._sameFace(pst):
                        self.delta[face.id] = pst.point
                except:
                    rospy.logwarn("No transform")
                    continue


    # Checks if we can merge two points from different cameras to
    # single face.
    # ps : PointStamped relative to the face we are checking
    def _sameFace(self, ps):
        # To be refined based on real data
        if math.sqrt(ps.point.x**2 + ps.point.y**2 + ps.point.z**2) <= self.max_distance:
            return True
        return False



if __name__ == '__main__':
    rospy.init_node('faces_tf2_broadcaster')
    FP = FacesPublisher()
    rospy.spin()
