# This module sets up a modal operator in Blender to act
# as the command listener for external events

commandRateHz = 10

import bpy
from . import network

class BLCommandListener(bpy.types.Operator):
	"""Operator which runs its self from a timer"""
	bl_label = "Command Listener"
	bl_idname = 'wm.command_listener'

	_timer = None
	# status = bpy.props.BoolProperty( name = "commandListenerActive", default=False)
	bpy.types.Scene.commandListenerActive = bpy.props.BoolProperty( name = "commandListenerActive", default=False)


	def modal(self, context, event):
		if event.type in {'ESC'}:
			return self.cancel(context)

		if event.type == 'TIMER':
			print('Running Command Listener', self._timer.time_duration)
			command = network.poll(context)


		bpy.context.scene['commandListenerActive'] = True
		return {'PASS_THROUGH'}


	def execute(self, context):
		success = network.init(context)
		if not success:
			print('Error connecting to external interface, stopping')
			return {'CANCELLED'}

		print('Starting Command Listener')
		wm = context.window_manager
		self._timer = wm.event_timer_add(1/commandRateHz, context.window)
		wm.modal_handler_add(self)
		bpy.context.scene['commandListenerActive'] = True
		return {'RUNNING_MODAL'}


	def cancel(self, context):
		print('Stopping Command Listener')
		wm = context.window_manager
		wm.event_timer_remove(self._timer)
		network.drop(context)
		bpy.context.scene['commandListenerActive'] = False
		return {'CANCELLED'}


def register():
	bpy.utils.register_class(BLCommandListener)


def unregister():
	bpy.utils.unregister_class(BLCommandListener)


def refresh():
	try:
		register()
	except Exception as E:
		print('re-registering')
		print(E)
		unregister()
		register()

	# test call
	# 
