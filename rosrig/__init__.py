# Initializes the ROS node

from . import roscom
node = None

def init(rigapi):
	global node
	node = roscom.build(rigapi)
	return node
