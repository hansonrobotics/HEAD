# This module sets up a modal operator in Blender to act
# as the animation playback service, and hosts other supporting test operators

framerateHz = 48

import bpy
from .helpers import *

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
	
	bpy.types.Scene.evaEmotion = bpy.props.StringProperty(name = "evaEmotion")
	bpy.context.scene['evaEmotion'] = "{'happy': {'magnitude': 0.9, 'duration': 10}}"

	def execute(self, context):
		from . import commands
		print(eval(self.action))
		return {'FINISHED'}



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
			
				eyeLoc = eva.eyeTargetLoc.target
				eyeLoc[0] = -normalX * 0.6
				eyeLoc[2] = normalY * 0.5

				headLoc = eva.headTargetLoc.target
				headLoc[0] = -normalX * 0.2
				headLoc[2] = normalY * 0.2

			# update NLA based gestures
			gestures = eva.gesturesList[:]  	# prevent in-situ removal while iterating bug
			for gesture in gestures:
				gesture.stripRef.strip_time += gesture.speed * timeScale

				if gesture.stripRef.strip_time > gesture.duration:
					if gesture.repeat > 1:
						gesture.repeat -= 1
						gesture.stripRef.strip_time = 0
					else:
						eva._deleteGesture(gesture)


			# update visemes
			visemes = eva.visemesList[:]
			for viseme in visemes:
				# wait to start
				if viseme.time < 0:
					continue

				# remove if finished (and finalized)
				if viseme.time > viseme.duration*1.5:
					eva._deleteViseme(viseme)
					continue
				
				# ramp in from 0
				rampPoint = viseme.duration * viseme.rampInRatio
				if viseme.time <= rampPoint:
					# compute ramp in factor
					viseme.magnitude.target = viseme.time / rampPoint

				# ramp out to 0
				rampOutPoint = viseme.duration - viseme.duration*viseme.rampOutRatio
				if viseme.time >= rampOutPoint:
					# compute ramp in factor
					viseme.magnitude.target = 1.0 - (viseme.time - rampOutPoint) / (viseme.duration*viseme.rampOutRatio)


				# update action
				viseme.magnitude.blend()
				viseme.stripRef.influence = viseme.magnitude.current

				# update time
				viseme.time += (1/framerateHz)*timeScale

				

			# update eye and head blending
			headControl = eva.bones["head_target"]
			eyeControl = eva.bones["eye_target"]

			eva.headTargetLoc.blend()
			eva.eyeTargetLoc.blend()

			headControl.location = eva.headTargetLoc.current
			eyeControl.location = eva.eyeTargetLoc.current
			
			# udpate emotions
			for emotion in eva.emotionsList:
				control = eva.bones['EMO-'+emotion.name]
				control['intensity'] = emotion.magnitude.current
				emotion.duration -= timeScale
				emotion.magnitude.blend()
				
				if emotion.duration < 0:
					emotion.magnitude._target *= 0.99

					if emotion.magnitude.current < 0.1:
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

			# send ROS data
			...

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
	bpy.utils.register_class(EvaDebug)


def unregister():
	bpy.utils.unregister_class(BLPlayback)
	bpy.utils.unregister_class(EvaDebug)


def refresh():
	try:
		register()
	except Exception as E:
		unregister()
		register()

