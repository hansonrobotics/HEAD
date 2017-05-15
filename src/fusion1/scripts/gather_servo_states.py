#!/usr/bin/env python2.7
import os
import rospy
import numpy
import logging
import time
import math

from sensor_msgs.msg import JointState as PololuJointState
from dynamixel_msgs.msg import JointState as DynamixelJointState
from fusion1.msg import ServoStates


class GatherServoStates(object):


    def __init__(self):

        # start subscribers and publishers
        self.states_pub = rospy.Publisher("servo_states",ServoStates,queue_size=5)
        self.pololu_sub = rospy.Subscriber("pololublabla",PololuJointState,HandlePololuJointState)
        self.dynamixel_sub = rospy.Subsriber("dynamixelblabla",DynamixelJointState,HandleDynamixelJointState)

        # also subscribe to PAU topics?

        ## Joint positions stright from PAU
        #self.pau_joint_names = rospy.get_param('~pau_joints', [])
        #if len(self.pau_joint_names) > 0:
        #    self.pau_joint_names = self.pau_joint_names.split(";")
        ## By default get PAU from blender
        #self.pau_topic = rospy.get_param('~pau_topic', "/blender_api/get_pau")
        ## Initialize publisher
        #self.joint_state_pub = rospy.Publisher("joint_states_combined", JointState, queue_size=10)
        #self.joint_state = JointState()
        #self.joint_state.header.frame_id = self.base_frame_id
        #self.joint_states_lock = RLock()

        # Subscribe to pololu servos
        #if len(self.pololu_joint_names) > 0:
        #    # Set the
        #    self.joint_state.name = self.pololu_joint_names
        #    num_pololu = len(self.pololu_joint_names)
        #    self.pololu_joint_positions = list(repeat(0.0, num_pololu))
        #   # rospy.Subscriber("pololu/motor_states", MotorStateList, self.update_pololu_joint_states)

        #if self.dyn_joint_names is not None:
        #    self.joint_state.name += self.dyn_joint_names
        #    num_dyn = len(self.dyn_joint_names)
        #    self.dyn_joint_positions = list(repeat(0.0, num_dyn))
        #    # Subscribe to dynamixels
        #    for name in self.dyn_joint_positions:
        #        controller = name.replace('_joint', '') + '_controller/state'
        #        logger.info('Subscribing to dynamixel controller: {0}'.format(controller))
        #        rospy.Subscriber(controller, DynamixelJointState, self.update_dyn_joint_state)

        #if len(self.pau_joint_names)>0:
        #    num = len(self.pau_joint_names)
        #    self.pau_joint_positions = list(repeat(0.0, num))
        #    self.joint_state.name += self.pau_joint_names
        #    rospy.Subscriber(self.pau_topic, pau, self.update_pau_joint_states)

        # start timer
        self.timer = rospy.Timer(rospy.Duration(0.1),self.HandleTimer)


    def HandlePololuJointState(self,data):

    	# TODO: update joint states related to Pololu


    def HandleDynamixelJointState(self,data):

    	# TODO: update joint states related to Dynamixel


    def HandleTimer(self,data):

    	# TODO: output that shit to fusion to update the transformations


if __name__ == '__main__':

    # initialize node
    rospy.init_node('gather_servo_states')

    # create age estimation object
    node = GatherServoStates()

    # run
    rospy.spin()
