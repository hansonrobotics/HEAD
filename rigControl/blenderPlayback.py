# This module sets up a modal operator in Blender to act
# as the animation playback service, and hosts other supporting test operators

framerateHz = 48
animationStep = 1.0

import bpy
from .helpers import *

import pprint

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

	bpy.types.Scene.evaEmotions = bpy.props.StringProperty(name = "evaEmotions")
	bpy.context.scene['evaEmotions'] = "{'happy':1, 'surprised':0.5}"
	
	
	def execute(self, context):
		# add to animationManager
		if self.evaEmotions == 'reset':
			bpy.evaAnimationManager.setEmotions('', reset = True)
		else:
			bpy.evaAnimationManager.setEmotions(eval(self.evaEmotions))
		
		return {'FINISHED'} 

	@classmethod
	def poll(cls, context):
		return bpy.context.scene['animationPlaybackActive']



class EvaTracking(bpy.types.Operator):
	"""Eva Tracking Control"""
	bl_idname = "eva.tracking"
	bl_label = "Eva Tracking"

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

			# compute mouse pos
			normalX = (event.mouse_region_x - 500) /1000
			normalY = (event.mouse_region_y - 500)/1000

			eyeLoc = bpy.evaAnimationManager.primaryEyeTargetLoc.target
			eyeLoc[0] = -normalX * 0.2
			eyeLoc[2] = normalY * 0.2


			# update NLA based gestures
			gestures = bpy.evaAnimationManager.gestureList
			for gesture in gestures:
				gesture.stripRef.strip_time += animationStep * gesture.speed

				if gesture.stripRef.strip_time > gesture.duration:
					if gesture.repeat > 1:
						gesture.repeat -= 1
						gesture.stripRef.strip_time = 0
					else:
						bpy.evaAnimationManager.deleteGesture(gesture)


			# update eye and head blending
			headControl = bpy.evaAnimationManager.bones["head_target"]
			eyeControl = bpy.evaAnimationManager.bones["eye_target"]

			bpy.evaAnimationManager.primaryHeadTargetLoc.blend()
			bpy.evaAnimationManager.primaryEyeTargetLoc.blend()

			headControl.location = bpy.evaAnimationManager.primaryHeadTargetLoc.current
			eyeControl.location = bpy.evaAnimationManager.primaryEyeTargetLoc.current
			
			# udpate emotions
			# pprint.pprint(bpy.evaAnimationManager.emotions)

			for emotionName, value in bpy.evaAnimationManager.emotions.items():
				control = bpy.evaAnimationManager.bones['EMO-'+emotionName]
				control['intensity'] = value.current
				value.target *= 0.99
				value.blend()


			# keep alive
			bpy.evaAnimationManager.keepAlive()

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
		bpy.evaAnimationManager.terminate()
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
		unregister()
		register()

