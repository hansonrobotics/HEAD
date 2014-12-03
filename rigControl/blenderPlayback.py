# This module sets up a modal operator in Blender to act
# as the animation playback service, and hosts other supporting test operators

framerateHz = 48

import bpy
from .helpers import *
from .animationSettings import *

import pprint, time

class EvaDebug(bpy.types.Operator):
	"""Eva Debug Control"""
	bl_idname = "eva.debug"
	bl_label = "Eva Debug"
	
	action = bpy.props.StringProperty()

	# register some helper bpy props for dev purposes
	bpy.types.Scene.evaFollowMouse = bpy.props.BoolProperty(name = "evaFollowMouse")
	bpy.context.scene['evaFollowMouse'] = False
	
	bpy.types.Scene.evaFPS = bpy.props.IntProperty(name = "evaFPS", soft_min = 10, soft_max = 60)
	bpy.context.scene['evaFPS'] = 0
	

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
		return bpy.context.scene['animationPlaybackActive'] and not bpy.context.scene['evaFollowMouse']



class BLPlayback(bpy.types.Operator):
	"""Playback Control"""
	bl_label = "Start Animation"
	bl_idname = 'wm.animation_playback'

	bpy.types.Scene.animationPlaybackActive = bpy.props.BoolProperty( name = "animationPlaybackActive", default=False)
	bpy.context.scene['animationPlaybackActive'] = False

	_timer = None

	timeList = []

	def modal(self, context, event):
		if event.type in {'ESC'}:
			return self.cancel(context)

		if event.type == 'TIMER':

			eva = bpy.evaAnimationManager

			# compute fps
			context.scene['evaFPS'] = fps = self.computeFPS(context)
			timeScale = framerateHz/fps

			if bpy.context.scene['evaFollowMouse']:
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
				gesture.stripRef.strip_time += gesture.speed * timeScale

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
			for emotion in eva.emotionsList:
				control = eva.bones['EMO-'+emotion.name]
				control['intensity'] = emotion.intensity.current
				emotion.duration -= timeScale
				emotion.intensity.blend()
				
				if emotion.duration < 0:
					emotion.intensity._target *= 0.99

					if emotion.intensity.current < 0.1:
						eva.emotionsList.remove(emotion)
						control['intensity'] = 0.0


			# Read emotion parameters into eva
			eva.eyeDartRate = eva.deformObj.pose.bones['eye_dart_rate']['value']
			eva.eyeWander = eva.deformObj.pose.bones['eye_wander']['value']
			eva.blinkRate = eva.deformObj.pose.bones['blink_rate']['value']
			eva.blinkDuration = eva.deformObj.pose.bones['blink_duration']['value']
			eva.breathRate = eva.deformObj.pose.bones['breath_rate']['value']
			eva.breathIntensity = eva.deformObj.pose.bones['breath_intensity']['value']

			# keep alive
			eva.keepAlive()

			# force update
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


	def computeFPS(self, context):
		# record
		self.timeList.append(time.time())

		# trim
		if len(self.timeList)> 100:
			self.timeList[100:]

		# compute
		counter = 0
		for timestamp in self.timeList:
			if timestamp > (time.time() - 1):
				counter += 1

		return counter


	@classmethod
	def poll(cls, context):
		return not bpy.context.scene['animationPlaybackActive']



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

