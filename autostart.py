#
# Autostart the blender animation.
#
# Run this as:
#    blender -y ./Eva269.blend -P ./autostart.py
#
import bpy

bpy.ops.wm.command_listener()

from rigControl import commands, CommandListener
commands.init()
# bpy.ops.eva.debug(action="commands.init")

# Start ROS node too..
# If ROS is found, we try to load the ROS node, else not.
import importlib
try:
	importlib.import_module('rospy')
	import rosrig

	node = rosrig.init()
	CommandListener.register_cmd_source(node)
except ImportError:
	print('No ROS found')
