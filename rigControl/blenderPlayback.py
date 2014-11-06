# This module sets up a modal operator in Blender to act
# as the animation playback service

framerateHz = 48

import bpy
from .helpers import *


class EvaGestures(bpy.types.Operator):
	"""Eva Gesture Control"""
	bl_idname = "eva.gestures"
	bl_label = "Eva Gestures"
	
	evaAction = bpy.props.StringProperty()
	
	def execute(self, context):
		# add to animationManager
		bpy.evaAnimationManager.newGesture(name=self.evaAction)
		return {'FINISHED'}

	@classmethod
	def poll(cls, context):
		return bpy.context.scene['animationPlaybackActive']



class EvaEmotions(bpy.types.Operator):
	"""Eva Emotion Control"""
	bl_idname = "eva.emotions"
	bl_label = "Eva Emotions"
	
	evaEmotions = bpy.props.StringProperty()
	
	def execute(self, context):
		# add to animationManager
		if self.evaEmotions == 'reset':
			bpy.evaAnimationManager.resetEmotions()
		else:
			bpy.evaAnimationManager.setEmotions({'happy':1, 'surprised':0.5})
		
		return {'FINISHED'} 

	@classmethod
	def poll(cls, context):
		return bpy.context.scene['animationPlaybackActive']



class EvaTracking(bpy.types.Operator):
	"""Eva Tracking Control"""
	bl_idname = "eva.tracking"
	bl_label = "Eva Tracking"
	
	bpy.types.Scene.evaTrackingTarget = bpy.props.FloatVectorProperty(subtype='DIRECTION')
	bpy.context.scene['evaTrackingTarget'] = [1.0,1.0,1.0]
	evaTrack = bpy.props.FloatVectorProperty()
	
	def execute(self, context):
		# add to animationManager
		bpy.evaAnimationManager.setPrimaryTarget(list(self.evaTrack))
		return {'FINISHED'} 

	@classmethod
	def poll(cls, context):
		return bpy.context.scene['animationPlaybackActive']



class BLPlayback(bpy.types.Operator):
	"""Playback Control"""
	bl_label = "Start Animation"
	bl_idname = 'wm.animation_playback'

	bpy.types.Scene.animationPlaybackActive = bpy.props.BoolProperty( name = "animationPlaybackActive", default=False)
	bpy.context.scene['animationPlaybackActive'] = False

	_timer = None


	def modal(self, context, event):
		if event.type in {'ESC'}:
			return self.cancel(context)

		if event.type == 'TIMER':

			# update NLA based gestures
			gestures = bpy.evaAnimationManager.gestureList
			for gesture in gestures:
				gesture.stripRef.strip_time += 1.0

				if gesture.stripRef.strip_time > gesture.duration:
					bpy.evaAnimationManager.deleteGesture(gesture)

			bpy.data.scenes['Scene'].frame_set(1)

			# update tracking
			headControl = bpy.data.objects['deform'].pose.bones["head"]
			eyeControl = bpy.data.objects['deform'].pose.bones["eye_target"]
			monkeyControl = bpy.data.objects['Suzanne']

			loc = bpy.evaAnimationManager.primaryTargetLoc
			
			eyeLoc = mix(list(eyeControl.location), loc, 0.9)
			headLoc = mix(list(headControl.location), loc, 0.9)
			monkeyLoc = mix(list(monkeyControl.location), loc, 0.9)

			# no effects when pose have keyframes
			# eyeControl.location = eyeLoc
			# headControl.location = headLoc

			monkeyControl.location = monkeyLoc

			bpy.data.scenes['Scene'].frame_set(1)

		return {'PASS_THROUGH'}


	def execute(self, context):
		print('Starting Playback')
		wm = context.window_manager
		self._timer = wm.event_timer_add(1/framerateHz, context.window)
		wm.modal_handler_add(self)
		bpy.context.scene['animationPlaybackActive'] = True
		return {'RUNNING_MODAL'}


	def cancel(self, context):
		print('Stopping Playback')
		wm = context.window_manager
		wm.event_timer_remove(self._timer)
		bpy.context.scene['animationPlaybackActive'] = False
		return {'CANCELLED'}


def register():
	bpy.utils.register_class(BLPlayback)
	bpy.utils.register_class(EvaGestures)
	bpy.utils.register_class(EvaEmotions)
	bpy.utils.register_class(EvaTracking)


def unregister():
	bpy.utils.unregister_class(BLPlayback)
	bpy.utils.unregister_class(EvaGestures)
	bpy.utils.unregister_class(EvaEmotions)
	bpy.utils.unregister_class(EvaTracking)


def refresh():
	try:
		register()
	except Exception as E:
		print('re-registering')
		print(E)
		unregister()
		register()

