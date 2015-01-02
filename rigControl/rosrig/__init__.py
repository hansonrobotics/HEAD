# Initializes the ROS node

from . import roscom
node = None

def init():
	global node
	node = roscom.build()
	return node
