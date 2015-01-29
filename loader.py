def register():
	import sys
	import bpy
	import imp

	# setup package path
	filePath = bpy.path.abspath('//')
	sys.path.append(filePath)

	# import package and force refresh for dev
	import rigControl
	imp.reload(rigControl)

	# Start ROS node too..
	# If ROS is found, we try to load the ROS node, else not.
	from rigControl import commands, CommandListener
	import importlib
	try:
		importlib.import_module('rospy')
		import rosrig

		node = rosrig.init(commands.EvaAPI())
		CommandListener.register_cmd_source(node)
	except ImportError:
		print('No ROS found')

register()
