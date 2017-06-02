#!/usr/bin/env python2.7
import os
import rospy
import numpy
import logging
import time
import math

from sensor_msgs.msg import JointState
from pau2motors.msg import pau as Pau


JOINT_NECK_ROLL = 0
JOINT_NECK_PITCH = 1
JOINT_NECK_YAW = 2
JOINT_HEAD_ROLL = 3
JOINT_HEAD_PITCH = 4
JOINT_EYES_PITCH = 5
JOINT_LEFT_EYE_YAW = 6
JOINT_RIGHT_EYE_YAW = 7


class GatherPAUStates(object):

    def __init__(self):

        # empty current joint state
        self.joint_state = JointState()
        self.joint_state.name = [None] * 8
        self.joint_state.position = [0.0] * 8
        self.joint_state.name[JOINT_NECK_ROLL] = "neck_roll"
        self.joint_state.name[JOINT_NECK_PITCH] = "neck_pitch"
        self.joint_state.name[JOINT_NECK_YAW] = "neck_yaw"
        self.joint_state.name[JOINT_HEAD_ROLL] = "head_roll"
        self.joint_state.name[JOINT_HEAD_PITCH] = "head_pitch"
        self.joint_state.name[JOINT_EYES_PITCH] = "eyes_pitch"
        self.joint_state.name[JOINT_LEFT_EYE_YAW] = "left_eye_yaw"
        self.joint_state.name[JOINT_RIGHT_EYE_YAW] = "right_eye_yaw"

        # start subscribers and publishers
        self.pau_subscriber = rospy.Subscriber("/blender_api/get_pau",Pau,self.HandlePAU)
        self.state_publisher = rospy.Publisher("/joint_states",JointState,queue_size=5)

        # start timer
        self.timer = rospy.Timer(rospy.Duration(0.1),self.HandleTimer)


    def HandlePAU(self,data):

        # head rotation is a quaternion, so convert to euler angles
        q = data.m_headRotation
        pitch = math.atan2(-2.0 * (q.y * q.z - q.w * q.x),q.w * q.w + q.y * q.y - q.z * q.z - q.x * q.x)
        yaw = math.asin(2.0 * (q.y * q.x + q.w * q.z))
        roll = math.atan2(-2.0 * (q.z * q.x - q.w * q.y),q.w * q.w - q.y * q.y - q.z * q.z + q.x * q.x)

        # head rotation is split over neck and head equally, and all yaws are inverted between Blender and URDF
        self.joint_state.position[JOINT_NECK_ROLL] = 0.5 * roll
        self.joint_state.position[JOINT_NECK_PITCH] = 0.5 * pitch
        self.joint_state.position[JOINT_NECK_YAW] = -yaw
        self.joint_state.position[JOINT_HEAD_ROLL] = 0.5 * roll
        self.joint_state.position[JOINT_HEAD_PITCH] = 0.5 * pitch
        self.joint_state.position[JOINT_EYES_PITCH] = data.m_eyeGazeLeftPitch
        self.joint_state.position[JOINT_LEFT_EYE_YAW] = -data.m_eyeGazeLeftYaw
        self.joint_state.position[JOINT_RIGHT_EYE_YAW] = -data.m_eyeGazeRightYaw


    def HandleTimer(self,data):

        # set timestamp
        self.joint_state.header.stamp = rospy.get_rostime()

        # and output the current joints
        self.state_publisher.publish(self.joint_state)


if __name__ == '__main__':

    rospy.init_node('gather_pau_states')
    node = GatherPAUStates()
    rospy.spin()
