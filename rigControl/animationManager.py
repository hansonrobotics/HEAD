from . import actuators
from .blendedNum import BlendedNum
from .helpers import *

import bpy	
import random
import time
import imp
import pdb

debug = True


class AnimationManager():
	
	def __init__(self):
		print('Starting AnimationManager singleton')
		
		# gesture params
		self.gesturesList = []
		self.emotionsList = []
		
		# tracking param
		self.primaryHeadTargetLoc = BlendedNum([0,0,0], steps=10, smoothing=10)
		self.secondaryHeadTargetLoc = BlendedNum([0,0,0], steps=10, smoothing=10)

		self.primaryEyeTargetLoc = BlendedNum([0,0,0], steps=4, smoothing=2)
		self.secondaryEyeTargetLoc = BlendedNum([0,0,0], steps=4, smoothing=2)

		# emotion params
		self.eyeDartRate = 0.0
		self.eyeWander = 0.0
		self.blinkRate = 0.0
		self.blinkDuration = 0.0
		self.breathRate = 0.0
		self.breathIntensity = 0.0

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
			
		if True:
			actuators.breathingCycle(self, self.breathRate, self.breathIntensity)

		if True and self.randomFrequency('dart', self.eyeDartRate):
			actuators.eyeSaccades(self, self.eyeWander)

		if True and self.randomFrequency('blink', self.blinkRate):
			actuators.blink(self, self.blinkDuration)

		if True and self.randomFrequency('primaryHeadTargetLoc', 1):
			actuators.headDrift(self)

		if True and self.randomFrequency('emotionJitter', 20):
			actuators.emotionJitter(self)
		


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


	def setEmotion(self, emotionDict):
		''' set the emotion param of the character'''
		for emotionName, data in emotionDict.items():
			try:
				control = self.bones['EMO-'+emotionName]
			except KeyError:
				print('Cannot set emotion. No bone with name ', emotionName)
				continue
			else:
				found = False
				for emotion in self.emotionsList:
					if emotionName == emotion.name:
						# update magnitude
						emotion.magnitude.target = data['magnitude']
						emotion.duration = data['duration']
						found = True
				
				if not found:
					emotion = Emotion(emotionName, magnitude = BlendedNum(data['magnitude'], steps = 10, smoothing = 10), duration = data['duration'])
					self.emotionsList.append(emotion)


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
		hz = max(hz, 0.0001)  # prevents div by 0
		if elapsedTime > random.gauss(1.0/hz, 0.2/hz):  # sigma is hard coded to 1/5 the mu
			self.lastTriggered[name] = time.time()
			return True
		else:
			return False



			
class Emotion():
	''' represents an emotion'''
	def __init__(self, name, magnitude, duration):
		self.name = name
		self.magnitude = magnitude
		self.duration = duration
		self.priority = 0

			
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



def init():
	'''Create AnimationManager singleton and make available for global access'''
	bpy.evaAnimationManager = AnimationManager()
