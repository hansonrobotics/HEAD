#! /usr/bin/env python
#
# Neck motors unit test.
#

# Set neck to an initial neutral position.

import rospy
import pau2motors.msg as paumsg


rospy.init_node('test_neck')

neck = rospy.Publisher("/blender_api/get_pau", paumsg.pau, queue_size=1, latch=True)

def neckrot(hx, hy, nx, ny) :
	msg = paumsg.pau()
	msg.m_headRotation.x = hx
	msg.m_headRotation.y = hy
	msg.m_headRotation.z = 0.0
	msg.m_headRotation.w = 1.0

	msg.m_neckRotation.x = nx
	msg.m_neckRotation.y = ny
	msg.m_neckRotation.z = 0.0
	msg.m_neckRotation.w = 1.0

	neck.publish(msg)

neckrot(0, 0, 0, 0)


for ir in range (0, 85) :
	r = ir * 0.001
	for it in range (0, 63, 2) :
		t = it * 0.1
		print "its ", r, t;

#
#rostopic pub --once /blender_api/get_pau  pau2motors/pau "m_headRotation:  
#  x: 0.0 
#  y: 0.085 
#  z: 0.0 
#  w: 0.9963809512430474 
#m_headTranslation:  
#  x: 0.0 
#  y: 0.0 
#  z: 0.0 
#m_neckRotation:  
#  x: 0.0 
#  y: 0.0 
#  z: 0.0 
#  w: 1.0 
#m_eyeGazeLeftPitch: -0.0216188617051 
#m_eyeGazeLeftYaw: -0.00167124893051 
#m_eyeGazeRightPitch: -0.0209033191204 
#m_eyeGazeRightYaw: -0.0445618629456 
#m_coeffs: [0.0, 0.75, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
#0.6212145090103149, 0.26078522205352783, 0.6212085485458374, 
#0.29769060015678406, 0.0, 0.0, 0.0, 0.0, 0.0075283292680978775, 0.0, 
#0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
#0.0, 0.0, 0.0, 0.0, 0.0]"
#
