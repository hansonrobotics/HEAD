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

	# Start command sources like ROS
	from rigControl import commands, CommandListener
	import pkg_resources
	group = 'blender_api.command_source.build'
	entry_points = list(pkg_resources.iter_entry_points(group))
	if len(entry_points) > 0:
		for point in entry_points:
			try:
				build = point.load()
				node = build(commands.EvaAPI())
				CommandListener.register_cmd_source(node)
				print("Command Source '%s' registered" % point.name)
			except ImportError:
				print("Command Source '%s' won't build" % point.name)
	else:
		print('No Command Sources found')

register()
