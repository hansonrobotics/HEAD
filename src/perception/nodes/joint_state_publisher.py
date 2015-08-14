#!/usr/bin/env python
#
# Robot joint motor control
# Copyright (C) 2014 OpenCog Foundation
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

#import ros_pololu_servo

__author__ = 'Alex van der Peet, James Diprose'

import rospy
from sensor_msgs.msg import JointState
from threading import Thread, RLock
#from ros_pololu.msg import MotorStateList
from dynamixel_msgs.msg import JointState as DynamixelJointState
from itertools import repeat 
from pau2motors.msg import pau
import math

class JointStatePublisher(Thread):
    NODE_NAME = 'joint_state_publisher_node'

    def __init__(self):
        super(JointStatePublisher, self).__init__()
        rospy.init_node(self.NODE_NAME)
        self.rate = rospy.Rate(rospy.get_param('~sensor_rate', 15.0))
        self.base_frame_id = rospy.get_param('~base_frame_id', "world")
        self.pololu_joint_names = rospy.get_param('~pololu_joints', [])
        self.pololu_joint_positions = []
        if len(self.pololu_joint_names) > 0:
          self.pololu_joint_names = self.pololu_joint_names.split(";")
        self.dyn_joint_names = rospy.get_param('~dyn_joints', None)
        # Joint positions stright from PAU
        self.pau_joint_names = rospy.get_param('~pau_joints', [])
        if len(self.pau_joint_names) > 0:
            self.pau_joint_names = self.pau_joint_names.split(";")
        # By default get PAU from blender
        self.pau_topic = rospy.get_param('~pau_topic', "/blender_api/get_pau")
        # Initialize publisher
        self.joint_state_pub = rospy.Publisher("joint_states_combined", JointState, queue_size=10)
        self.joint_state = JointState()
        self.joint_state.header.frame_id = self.base_frame_id

        self.joint_states_lock = RLock()

        # Subscribe to pololu servos
        if len(self.pololu_joint_names) > 0:
            # Set the
            self.joint_state.name = self.pololu_joint_names
            num_pololu = len(self.pololu_joint_names)
            self.pololu_joint_positions = list(repeat(0.0, num_pololu))
           # rospy.Subscriber("pololu/motor_states", MotorStateList, self.update_pololu_joint_states)

        if self.dyn_joint_names is not None:
            self.joint_state.name += self.dyn_joint_names
            num_dyn = len(self.dyn_joint_names)
            self.dyn_joint_positions = list(repeat(0.0, num_dyn))
            # Subscribe to dynamixels
            for name in self.dyn_joint_positions:
                controller = name.replace('_joint', '') + '_controller/state'
                rospy.loginfo('Subscribing to dynamixel controller: {0}'.format(controller))
                rospy.Subscriber(controller, DynamixelJointState, self.update_dyn_joint_state)

        if len(self.pau_joint_names)>0:
            num = len(self.pau_joint_names)
            self.pau_joint_positions = list(repeat(0.0, num))
            self.joint_state.name += self.pau_joint_names
            rospy.Subscriber(self.pau_topic, pau, self.update_pau_joint_states)
        # Quternion 2 Euler convertion
        self._quaternion_to_euler = {
          'y': lambda q: math.atan2(
            -2*(q.z*q.x-q.w*q.y),
            q.w**2-q.y**2-q.z**2+q.x**2
          ),
          'z': lambda q: math.asin(
            2*(q.y*q.x + q.w*q.z)
          ),
          'x': lambda q: math.atan2(
            -2*(q.y*q.z-q.w*q.x),
            q.w**2+q.y**2-q.z**2-q.x**2
          )
        }



    def update_dyn_joint_state(self, msg):
        with self.joint_states_lock:
            joint_index = self.dyn_joint_names.index(msg.name)
            self.dyn_joint_positions[joint_index] = msg.current_pos

    def update_pololu_joint_states(self, msg):
        # Get new positions
        with self.joint_states_lock:
            # we only need shortlist of joints
            for motor_state in msg.motor_states:
                if motor_state.name in self.pololu_joint_names:
                    self.pololu_joint_positions[self.pololu_joint_names.index(motor_state.name)] = motor_state.radians

    def update_pau_joint_states(self, msg):

        with self.joint_states_lock:
            # Head pitch devided between two joints
           
            if('pitch_base_joint' in self.pau_joint_names):
                angle = self._quaternion_to_euler['x'](msg.m_headRotation) / 2.0
                self.pau_joint_positions[self.pau_joint_names.index('pitch_base_joint')] = angle
                print angle
            if('pitch_neck_joint' in self.pau_joint_names):
                angle = self._quaternion_to_euler['x'](msg.m_headRotation) / 2.0
                self.pau_joint_positions[self.pau_joint_names.index('pitch_neck_joint')] = angle
            # One Yaw Joint. The Yaw for head and eyes are inverted  compared to URDF
            if('yaw_joint' in self.pau_joint_names):
                angle = -self._quaternion_to_euler['z'](msg.m_headRotation)
                self.pau_joint_positions[self.pau_joint_names.index('yaw_joint')] = angle
            # Single eyes pitch joint
            if('Eyes_Pitch' in self.pau_joint_names):
                angle = msg.m_eyeGazeLeftPitch
                self.pau_joint_positions[self.pau_joint_names.index('Eyes_Pitch')] = angle
            if('Eye_L' in self.pau_joint_names):
                angle = -msg.m_eyeGazeLeftYaw
                self.pau_joint_positions[self.pau_joint_names.index('Eye_L')] = angle
            if('Eye_R' in self.pau_joint_names):
                angle = -msg.m_eyeGazeLeftYaw
                self.pau_joint_positions[self.pau_joint_names.index('Eye_R')] = angle
            print self.pau_joint_positions
    def run(self):
        while not rospy.is_shutdown():
            with self.joint_states_lock:
                self.joint_state.header.stamp = rospy.Time.now()
                self.joint_state.position = []
                # Complete JointState message
                if self.dyn_joint_names is not None and self.pololu_joint_names is not None:
                    self.joint_state.position = self.pololu_joint_positions + self.dyn_joint_positions

                elif self.dyn_joint_names is not None:
                    self.joint_state.position = self.dyn_joint_positions

                elif self.pololu_joint_names is not None:
                    self.joint_state.position = self.pololu_joint_positions
                # Add PAU positions
                self.joint_state.position = self.pau_joint_positions

                # Publish JointState messags
                self.joint_state_pub.publish(self.joint_state)

            self.rate.sleep()

if __name__ == '__main__':
	publisher = JointStatePublisher()
	publisher.run()
