#! /usr/bin/env python
#
# Neck motors unit test.
#
# To test, run this by hand at the command line.  The head should
# spiral around doing an egyptian/persian neck dance, and then return
# to center.
#
# Linas Vepstas July 2015
#

import math
import rospy
import time
import pau2motors.msg as paumsg

# Create a new ros node for the test...
rospy.init_node('test_neck')
neck = rospy.Publisher("/blender_api/get_pau", paumsg.pau, queue_size=1)
# Give the ros node some time to get registered with roscore and any
# subscribers.
time.sleep(1)

# Neck rotation --- create some quaternions, quick-n-dirty hack.
def neckrot(hx, hy, nx, ny) :
	global neck
	msg = paumsg.pau()
	msg.m_headRotation.x = hx
	msg.m_headRotation.y = hy
	msg.m_headRotation.z = 0.0
	msg.m_headRotation.w = math.sqrt(1.0 - hx*hx + hy*hy)

	msg.m_neckRotation.x = nx
	msg.m_neckRotation.y = ny
	msg.m_neckRotation.z = 0.0
	msg.m_neckRotation.w = math.sqrt(1.0 - nx*nx + ny*ny)

	neck.publish(msg)

# Set neck to an initial neutral position.
neckrot(0, 0, 0, 0)

# Rotate the neck in a spiral
for ir in range (5, 85, 10) :
	r = ir * 0.001
	for it in range (0, 63, 2) :
		t = it * 0.1
		print "radius=%7f phi=%7f " % (r , t)
		hx = r * math.cos(t)
		hy = r * math.sin(t)
		nx = -hx
		ny = -hy
		neckrot(hx, hy, nx, ny)

		time.sleep(0.2)

neckrot(0, 0, 0, 0)
time.sleep(1)
print "Done with the neck test"
