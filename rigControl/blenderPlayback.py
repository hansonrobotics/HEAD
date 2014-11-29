# This module sets up a modal operator in Blender to act
# as the animation playback service, and hosts other supporting test operators

framerateHz = 48
animationStep = 1.0

import bpy
from .helpers import *
from .animationSettings import *


import pprint

class EvaDebug(bpy.types.Operator):
	"""Eva Debug Control"""
	bl_idname = "eva.debug"
	bl_label = "Eva Debug"
	
	action = bpy.props.StringProperty()
	
	def execute(self, context):
		from . import commands
		print(eval(self.action))
		return {'FINISHED'}

	@classmethod
	def poll(cls, context):
		return bpy.context.scene['animationPlaybackActive']

	


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
	bpy.context.scene['evaEmotions'] = "{'happy':0.5, 'amused':0.5}"
	
	
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

			eva = bpy.evaAnimationManager

			# compute mouse pos
			normalX = (event.mouse_region_x - 500) /1000
			normalY = (event.mouse_region_y - 500)/1000

			eyeLoc = eva.primaryEyeTargetLoc.target
			eyeLoc[0] = -normalX * 0.6
			eyeLoc[2] = normalY * 0.5

			headLoc = eva.primaryHeadTargetLoc.target
			headLoc[0] = -normalX * 0.2
			headLoc[2] = normalY * 0.2

			# update NLA based gestures
			gestures = eva.gesturesList
			for gesture in gestures:
				gesture.stripRef.strip_time += animationStep * gesture.speed

				if gesture.stripRef.strip_time > gesture.duration:
					if gesture.repeat > 1:
						gesture.repeat -= 1
						gesture.stripRef.strip_time = 0
					else:
						eva._deleteGesture(gesture)


			# update eye and head blending
			headControl = eva.bones["head_target"]
			eyeControl = eva.bones["eye_target"]

			eva.primaryHeadTargetLoc.blend()
			eva.primaryEyeTargetLoc.blend()

			headControl.location = eva.primaryHeadTargetLoc.current
			eyeControl.location = eva.primaryEyeTargetLoc.current
			
			# udpate emotions
			for emotionName, value in eva.emotions.items():
				control = eva.bones['EMO-'+emotionName]
				control['intensity'] = value.current
				value._target *= 0.99
				value.blend()

			# Read emotion parameters into eva
			eyeDartRate = eva.deformObj.pose.bones['eye_dart_rate'].location[0]
			eyeWander = eva.deformObj.pose.bones['eye_wander'].location[0]
			blinkRate = eva.deformObj.pose.bones['blink_rate'].location[0]
			blinkDuration = eva.deformObj.pose.bones['blink_duration'].location[0]
			breathRate = eva.deformObj.pose.bones['breath_rate'].location[0]
			breathIntensity = eva.deformObj.pose.bones['breath_intensity'].location[0]

			eva.eyeDartRate = mapValue(eyeDartRate, -1, 1, MIN_EYE_DART_RATE, MAX_EYE_DART_RATE)
			eva.eyeWander = mapValue(eyeWander, 0, 1, MIN_EYE_WANDER, MAX_EYE_WANDER)
			eva.blinkRate = mapValue(blinkRate, -1, 3, MIN_BLINK_RATE, MAX_BLINK_RATE)
			eva.blinkDuration = blinkDuration
			eva.breathRate = mapValue(breathRate, -1, 1, MIN_BREATH_RATE, MAX_BREATH_RATE)
			eva.breathIntensity = mapValue(breathIntensity, -1, 1, MIN_BREATH_INTENSITY, MAX_BREATH_INTENSITY)


			# keep alive
			eva.keepAlive()

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
	bpy.utils.register_class(EvaDebug)


def unregister():
	bpy.utils.unregister_class(BLPlayback)
	bpy.utils.unregister_class(EvaGestures)
	bpy.utils.unregister_class(EvaEmotions)
	bpy.utils.unregister_class(EvaTracking)
	bpy.utils.unregister_class(EvaDebug)


def refresh():
	try:
		register()
	except Exception as E:
		unregister()
		register()

