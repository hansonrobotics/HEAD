#! /usr/bin/env python
#
# Neck motors unit test.
#

# Set neck to an initial neutral position.

import math
import rospy
import pau2motors.msg as paumsg


rospy.init_node('test_neck')

neck = rospy.Publisher("/blender_api/get_pau", paumsg.pau, queue_size=1, latch=True)

def neckrot(hx, hy, nx, ny) :
	global neck
	msg = paumsg.pau()
	msg.m_headRotation.x = hx
	msg.m_headRotation.y = hy
	msg.m_headRotation.z = 0.0
	msg.m_headRotation.w = 1.0 - math.sqrt(hx*hx + hy*hy)

	msg.m_neckRotation.x = nx
	msg.m_neckRotation.y = ny
	msg.m_neckRotation.z = 0.0
	msg.m_neckRotation.w = 1.0 - math.sqrt(nx*nx + ny*ny)

	print "duuude wtf"
	neck.publish(msg)

neckrot(0, 0, 0, 0.1)

exit (1)

for ir in range (0, 85, 10) :
	r = ir * 0.001
	for it in range (0, 63, 2) :
		t = it * 0.1
		print "radius=%7f phi=%7f " % (r , t)
		hx = r * math.cos(t)
		hy = r * math.sin(t)
		nx = -hx
		ny = -hy
		neckrot(hx, hy, nx, ny)

		
neckrot(0, 0, 0, 0)

