# This module sets up a modal operator in Blender to act
# as the animation playback service

framerateHz = 48

import bpy


def emotionCycle(context, playIndex):
	bones = bpy.data.objects['control'].pose.bones
	emo = bones[playIndex%len(bones)]
	if "EMO" in emo.name:
		if emo['intensity'] < 1:
			emo['intensity'] += 0.01
			return playIndex
			
		elif emo['intensity'] >= 1:
			emo['intensity'] = 0
			playIndex += 1
			return playIndex

	return 0




class BLPlayback(bpy.types.Operator):
	"""Operator which runs its self from a timer"""
	bl_label = "Animation Playback"
	bl_idname = 'wm.animation_playback'

	_timer = None
	playIndex = 0


	def modal(self, context, event):
		if event.type in {'ESC'}:
			return self.cancel(context)

		if event.type == 'TIMER':
			# print('Running Animation:', self._timer.time_duration)
			self.playIndex = emotionCycle(context, self.playIndex)

		return {'PASS_THROUGH'}


	def execute(self, context):
		print('Starting Playback')
		wm = context.window_manager
		self._timer = wm.event_timer_add(1/framerateHz, context.window)
		wm.modal_handler_add(self)
		bpy.ops.eva.looper()
		return {'RUNNING_MODAL'}


	def cancel(self, context):
		print('Stopping Playback')
		wm = context.window_manager
		wm.event_timer_remove(self._timer)
		return {'CANCELLED'}


def register():
	bpy.utils.register_class(BLPlayback)


def unregister():
	bpy.utils.unregister_class(BLPlayback)


def refresh():
	try:
		register()
	except Exception as E:
		print('re-registering')
		print(E)
		unregister()
		register()

