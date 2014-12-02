# deals with interface with other apps

from . import roscom
rosnode = None


def init(context):
	global rosnode
	rosnode = roscom.build()
	success = bool(rosnode)
	return success


def poll(context):
	command = rosnode.poll()
	return command


def drop(context):
	success = rosnode.drop()
	return success
