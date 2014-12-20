# deals with interface with other apps

from . import roscom
node = None

def init():
	global node
	node = roscom.build()
	return node
