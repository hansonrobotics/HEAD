# This module sets up a modal operator in Blender to act
# as the command listener for external events

commandRateHz = 1

import bpy

class BLCommandListener(bpy.types.Operator):
	"""Operator which runs its self from a timer"""
	bl_label = "Command Listener"
	bl_idname = 'wm.command_listener'
	_timer = None

	def modal(self, context, event):
		if event.type in {'RIGHTMOUSE', 'ESC'}:
			print('Stopping Command Listener')
			return self.cancel(context)

		if event.type == 'TIMER':
			print('Running Command Listener', self._timer.time_duration)

		return {'PASS_THROUGH'}

	def execute(self, context):
		wm = context.window_manager
		self._timer = wm.event_timer_add(1/commandRateHz, context.window)
		wm.modal_handler_add(self)
		return {'RUNNING_MODAL'}

	def cancel(self, context):
		wm = context.window_manager
		wm.event_timer_remove(self._timer)
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
