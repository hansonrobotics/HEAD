from . import actuators
from .helpers import *

import bpy	
import random
import time

class AnimationManager():
	
	def __init__(self):
		print('Starting AnimationManager singleton')
		self.gestureList = []
		
		self.primaryHeadTargetLoc = BlendedNum([0,0,0], steps=50)
		self.primaryEyeTargetLoc = BlendedNum([0,0,0], steps=5)
		self.secondaryHeadTargetLoc = BlendedNum([0,0,0], steps=50)
		self.secondaryEyeTargetLoc = BlendedNum([0,0,0], steps=5)

		# emotino params
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



	def keepAlive(self):
		self.idle += 1.0

		# actuators.idleCycle(self)
		# actuators.breathingCycle(self)

		if True and self.randomFrequency('primaryEyeTargetLoc', 2):
			actuators.eyeSaccades(self)

		if True and self.randomFrequency('blink', 0.1):
			self.newGesture('GST-blink')

		if False and self.randomFrequency('primaryHeadTargetLoc', 1):
			# randomly adjust head target
			loc = [0,0,0]
			loc[0] = random.gauss(self.primaryHeadTargetLoc.current[0], 0.01)
			loc[1] = random.gauss(self.primaryHeadTargetLoc.current[1], 0.01)
			loc[2] = random.gauss(self.primaryHeadTargetLoc.current[2], 0.02)
			self.primaryHeadTargetLoc.target = loc


	# show all attributes
	def __repr__(self):
		string = ""
		for attr, value in sorted(self.__dict__.items()):
			if not attr.startswith('_'):
				string += str(attr) + ": " + str(value) + "\n"
		return string


	def newGesture(self, name, repeat = 1, speed=1, magnitude=0.5, priority=1):
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
		self.gestureList.append(g)


	def deleteGesture(self, gesture):
		# remove from list
		self.gestureList.remove(gesture)

		# remove from Blender
		self.deformObj.animation_data.nla_tracks.remove(gesture.trackRef)


	def setEmotions(self, emotions):
		checkValue(emotions, -1, 1)

		for emotion, value in emotions.items():
			try:
				control = self.bones['EMO-'+emotion]
			except KeyError:
				print('Cannot set emotion. No bone with name ', emotion)
				continue
			else:
				control['intensity'] = float(value)


	def resetEmotions(self):
		bones = bpy.data.objects['control'].pose.bones
		for bone in self.bones:
			if bone.name.startswith('EMO-'):
				bone['intensity'] = 0.0
		

	def setPrimaryTarget(self, loc):
		# compute distance from previous eye position
		distance = computeDistance(loc, self.primaryEyeTargetLoc.current)

		if distance > 0.15:
			if self.randomFrequency('blink', 20):
				self.newGesture('GST-blink-micro')

		self.primaryHeadTargetLoc.target = loc
		self.primaryEyeTargetLoc.target = loc
			

	def randomFrequency(self, name, hz):
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
	
	def __init__(self, name, track, strip, duration=100, speed=1, magnitude=1, priority=1, repeat=1):
		self.name = name
		self.duration = duration
		self.magnitude = magnitude
		self.speed = speed
		self.priority = priority
		self.repeat = repeat

		self.trackRef = track
		self.stripRef = strip


class BlendedNum():
	def __init__(self, value, steps = 50):
		self._target = value.copy()
		self._old = value.copy()
		self._current = value.copy()
		self._steps = steps

	def __repr__(self):
		allStr = 'BlendedNum:' + str((self._current, self._old, self._target))
		return allStr

	@property
	def current(self):
		return self._current
	@current.setter
	def current(self, value):
		self._current = value


	@property
	def target(self):
		return self._target

	@target.setter
	def target(self, value):
		self._target = value.copy()
	
	@property
	def steps(self):
		return self._steps
	@steps.setter
	def steps(self, value):
		self._steps = value
	
	def blend(self):
		if type(self._target) is list and len(self._target)==3:
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
			deltaValue = (self._target - self._old) / self._steps
			self._current += deltaValue
			
			if (deltaValue > 0 and self._current > self._target) or (deltaValue < 0 and self._current < self._target):
				self._current = self._target.copy()
				self._old = self._target.copy()
				return


def init():
	if bpy:
		bpy.evaAnimationManager = AnimationManager()
	else:
		return AnimationManager()