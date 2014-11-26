from . import actuators
from .helpers import *


import bpy	
import random
import time
import imp
import pdb
import collections

debug = True

class AnimationManager():
	
	def __init__(self):
		print('Starting AnimationManager singleton')
		
		# gesture params
		self.gesturesList = []
		
		# tracking param
		self.primaryHeadTargetLoc = BlendedNum([0,0,0], steps=10, smoothing=10)
		self.secondaryHeadTargetLoc = BlendedNum([0,0,0], steps=10, smoothing=10)

		self.primaryEyeTargetLoc = BlendedNum([0,0,0], steps=4, smoothing=2)
		self.secondaryEyeTargetLoc = BlendedNum([0,0,0], steps=4, smoothing=2)


		# emotion params
		self.emotions = {}
		self.blinkFrequency = 1.0
		self.blinkSpeed = 1.0
		self.breathingSpeed = 1.0
		self.breathingIntensity = 1.0
		self.swiftness = 1.0
		self.shyness = 1.0
		self.idle = 0.0

		# internal vars
		self._time = 0
		self.lastTriggered = {}

		# global access
		self.deformObj = bpy.data.objects['deform']
		self.bones = bpy.data.objects['control'].pose.bones

		if debug:
			imp.reload(actuators)


	def keepAlive(self):
		''' called every frame, used to dispatch animation actuators'''
		self.idle += 1.0

		if True:
			actuators.idleCycle(self)
			
		if False:
			actuators.breathingCycle(self)

		if True and self.randomFrequency('primaryEyeTargetLoc', 2):
			actuators.eyeSaccades(self)

		if True and self.randomFrequency('blink', 0.1):
			self.newGesture('GST-blink')

		if True and self.randomFrequency('primaryHeadTargetLoc', 1):
			actuators.headDrift(self)
			

	# show all attributes
	def __repr__(self):
		string = ""
		for attr, value in sorted(self.__dict__.items()):
			if not attr.startswith('_'):
				string += str(attr) + ": " + str(value) + "\n"
		return string


	def newGesture(self, name, repeat = 1, speed=1, magnitude=0.5, priority=1):
		''' create a new gesture '''
		try:
			actionDatablock = bpy.data.actions[name]
		except KeyError:
			raise Exception('No gesture matching name is found')
			return

		# check value for sanity
		checkValue(repeat, 1, 1000)
		checkValue(speed, 0.1, 10)
		checkValue(magnitude, 0, 1)
		checkValue(priority, 0, 1)

		# create NLA track
		newTrack = self.deformObj.animation_data.nla_tracks.new()
		newTrack.name = name

		# create strip
		newStrip = newTrack.strips.new(name=name, start=1, action=actionDatablock)
		duration = (newStrip.frame_end - newStrip.frame_start)
		newStrip.blend_type = 'ADD'
		newStrip.use_animated_time = True

		# force blink to play at 1.0 intensity
		if 'blink' in name.lower():
			magnitude = 1

		if magnitude < 1:
			newStrip.use_animated_influence = True
			newStrip.influence = magnitude

		# create object and add to list
		g = Gesture(name, newTrack, newStrip, duration=duration, speed=speed, magnitude=magnitude, priority=priority, repeat=repeat)
		self.gesturesList.append(g)


	def _deleteGesture(self, gesture):
		''' internal use only, stops and deletes a gesture'''
		# remove from list
		self.gesturesList.remove(gesture)

		# remove from Blender
		self.deformObj.animation_data.nla_tracks.remove(gesture.trackRef)


	def setEmotions(self, emotions, reset=False):
		''' set the emotion param of the character'''
		for emotionName, value in emotions.items():
			try:
				control = self.bones['EMO-'+emotionName]
			except KeyError:
				print('Cannot set emotion. No bone with name ', emotionName)
				continue
			else:
				value = float(value)
				checkValue(value, -1, 1)
				if emotionName in self.emotions:
					self.emotions[emotionName].target = value
				else:
					self.emotions[emotionName] = BlendedNum(value, steps = 10, smoothing = 10)


	def setPrimaryTarget(self, loc):
		''' set the target used by eye and face tracking '''
		# compute distance from previous eye position
		distance = computeDistance(loc, self.primaryEyeTargetLoc.current)

		if distance > 0.15:
			if self.randomFrequency('blink', 20):
				self.newGesture('GST-blink-micro')

		self.primaryHeadTargetLoc.target = loc
		self.primaryEyeTargetLoc.target = loc
			

	def terminate(self):
		''' house keeping at the end of the run'''
		# remove all leftover gestures
		for gesture in self.gesturesList:
			self.deformObj.animation_data.nla_tracks.remove(gesture.trackRef)

		self.gesturesList = []

		# reset pose
		bpy.context.scene.objects.active = self.deformObj
		try:
			bpy.ops.pose.transforms_clear()
		except:
			bpy.ops.object.posemode_toggle()
			bpy.ops.pose.transforms_clear()


	def randomFrequency(self, name, hz):
		''' returns a random true/false based on a hertz value as input'''
		try:
			oldTime = self.lastTriggered[name]
		except KeyError:
			self.lastTriggered[name] = time.time()
			return True

		elapsedTime = time.time() - oldTime
		if elapsedTime > random.gauss(1.0/hz, 0.2/hz):  # sigma is hard coded to 1/5 the mu
			self.lastTriggered[name] = time.time()
			return True
		else:
			return False



			
class Gesture():
	''' represents a blender actions'''
	def __init__(self, name, track, strip, duration, speed, magnitude, priority, repeat):
		self.name = name
		self.duration = duration
		self.magnitude = magnitude
		self.speed = speed
		self.priority = priority
		self.repeat = repeat

		self.trackRef = track
		self.stripRef = strip


class BlendedNum():
	''' represents a number or vec3 float that is smoothly blended'''
	def __init__(self, value, steps = 20, smoothing = 20):
		if type(value) is float or type(value) is int:
			self._target = value
			self._old = value
			self._current = value
			self._movingAverage = collections.deque(5*[0], smoothing)
			self.isVector = False
		else:
			self._target = value.copy()
			self._old = value.copy()
			self._current = value.copy()
			self._movingAverage = collections.deque(5*[[0,0,0]], smoothing)
			self.isVector = True
			
		self._steps = steps
		

	def __repr__(self):
		allStr = 'BlendedNum:' + str((self._current, self._old, self._target))
		return allStr

	@property
	def current(self):
		''' return moving average instead of raw lerp value '''
		if self.isVector:
			return [float(sum(col))/len(col) for col in zip(*list(self._movingAverage))]
		else:
			return sum(self._movingAverage)/len(self._movingAverage)

	@current.setter
	def current(self, value):
		self._current = value


	@property
	def target(self):
		return self._target

	@target.setter
	def target(self, value):
		if self.isVector:
			self._target = value.copy()
			self._old = self._current.copy()
		else:
			self._target = value
			self._old = self._current
			
	
	@property
	def steps(self):
		return self._steps
	@steps.setter
	def steps(self, value):
		self._steps = value
	

	def blend(self):
		#store old value to moving average:
		if self.isVector:
			self._movingAverage.append(self._current.copy())
		else:
			self._movingAverage.append(self._current)

		if self.isVector and len(self._target)==3:
			# trilinear interpolation
			delta = [0,0,0]
			delta[0] = (self._target[0] - self._old[0]) / self._steps
			delta[1] = (self._target[1] - self._old[1]) / self._steps
			delta[2] = (self._target[2] - self._old[2]) / self._steps

			self._current[0] += delta[0]
			self._current[1] += delta[1]
			self._current[2] += delta[2]
			
			if (delta[0] > 0 and self._current[0] > self._target[0]) or (delta[0] < 0 and self._current[0] < self._target[0]):
				self._current[0] = self._target[0]
				self._old[0] = self._target[0]
			if (delta[1] > 0 and self._current[1] > self._target[1]) or (delta[1] < 0 and self._current[1] < self._target[1]):
				self._current[1] = self._target[1]
				self._old[1] = self._target[1]
			if (delta[2] > 0 and self._current[2] > self._target[2]) or (delta[2] < 0 and self._current[2] < self._target[2]):
				self._current[2] = self._target[2]
				self._old[2] = self._target[2]
			
		else:
			# linear interpolation
			deltaValue = (self._target - self._old) / self._steps
			self._current += deltaValue
			
			if (deltaValue > 0 and self._current > self._target) or (deltaValue < 0 and self._current < self._target):
				self._current = self._target
				self._old = self._target
				return


def init():
	bpy.evaAnimationManager = AnimationManager()
