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



class EvaDriver(bpy.types.Operator):
	bl_idname = "eva.driver"
	bl_label = "Gestures"
	
	evaAction = bpy.props.StringProperty()
	
	def execute(self, context):
		# run gesture here
		deformObj = bpy.data.objects['deform']
		actionDatablock = bpy.data.actions[self.evaAction]

		# create NLA track
		newTrack = deformObj.animation_data.nla_tracks.new()
		newTrack.name = self.evaAction

		# create strip
		actionName = self.evaAction
		action = actionDatablock
		curFrame = 1
		newStrip = newTrack.strips.new(name=actionName, start=curFrame, action=action)
		newStrip.blend_type = 'ADD'
		newStrip.use_animated_time = True
		
		# add to animationManager
		bpy.evaAnimationManager.newGesture(name=self.evaAction, track=newTrack, strip=newStrip)
		
		print(len(bpy.evaAnimationManager.gestureList))

		return {'FINISHED'} 



class BLPlayback(bpy.types.Operator):
	"""Operator which runs its self from a timer"""
	bl_label = "Animation Playback"
	bl_idname = 'wm.animation_playback'

	option = bpy.props.StringProperty()


	_timer = None
	playIndex = 0


	def modal(self, context, event):
		if event.type in {'ESC'}:
			return self.cancel(context)

		if event.type == 'TIMER':
			# print('Running Animation:', self._timer.time_duration)
			# update emotion
			if 'emo' in self.option:
				self.playIndex = emotionCycle(context, self.playIndex)

			# update gestures
			if 'ges' in self.option:
				gestures = bpy.evaAnimationManager.gestureList
				for gesture in gestures:
					gesture.stripRef.strip_time += 1.0
					bpy.data.scenes['Scene'].frame_set(1)

					if gesture.stripRef.strip_time > gesture.duration:
						bpy.evaAnimationManager.deleteGesture(gesture)



		return {'PASS_THROUGH'}


	def execute(self, context):
		print('Starting Playback')
		wm = context.window_manager
		self._timer = wm.event_timer_add(1/framerateHz, context.window)
		wm.modal_handler_add(self)
		# bpy.ops.wm.looper()
		return {'RUNNING_MODAL'}


	def cancel(self, context):
		print('Stopping Playback')
		wm = context.window_manager
		wm.event_timer_remove(self._timer)
		return {'CANCELLED'}


def register():
	bpy.utils.register_class(BLPlayback)
	bpy.utils.register_class(EvaDriver)


def unregister():
	bpy.utils.unregister_class(BLPlayback)
	bpy.utils.unregister_class(EvaDriver)


def refresh():
	try:
		register()
	except Exception as E:
		print('re-registering')
		print(E)
		unregister()
		register()

