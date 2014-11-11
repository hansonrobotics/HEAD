try:
	import bpy
except:
	bpy = None
	print ('Running without Blender')

from .helpers import *

import random

class AnimationManager():
	
	def __init__(self):
		print('Starting AnimationManager singleton')
		self.gestureList = []
		
		self.primaryHeadTargetLoc = [0,0,0]
		self.primaryEyeTargetLoc = [0,0,0]
		self.secondaryHeadTargetLoc = [0,0,0]
		self.secondaryEyeTargetLoc = [0,0,0]


		self.blinkFrequency = 1.0
		self.blinkSpeed = 1.0
		self.breathingFrequency = 1.0
		self.breathingIntensity = 1.0
		self.swiftness = 1.0
		self.shyness = 1.0
		self.idle = 0.0

		# internal vars
		self._time = 0

		self._primaryHeadTargetLoc = [0,0,0]
		self._primaryEyeTargetLoc = [0,0,0]
		self._secondaryHeadTargetLoc = [0,0,0]
		self._secondaryEyeTargetLoc = [0,0,0]

		self._blend = 0


	def keepAlive(self):
		self.idle += 1.0


		# idle cycle
		for loopBone in bpy.data.objects['control'].pose.bones:
			if "CYC" in loopBone.name: 
				if loopBone.location[1] < 1:
					loopBone.location[1] += 1/(4*loopBone['frames'])
				else :
					loopBone.location[1] = 0


		bpy.data.objects['control'].pose.bones["CYC-normal"]["influence"] = 1.0



		# global frequency
		if random.random() > 0.3:
			return

		if random.random() < 0.1:
			# randomly adjust eye target
			loc = [0,0,0]
			loc[0] = random.gauss(self.primaryEyeTargetLoc[0], 0.05)
			loc[1] = random.gauss(self.primaryEyeTargetLoc[1], 0.02)
			loc[2] = random.gauss(self.primaryEyeTargetLoc[2], 0.02)
			self._primaryEyeTargetLoc = loc

		if random.random() < 0.01:
			# blink
			self.newGesture('GST-blink')

		
		if random.random() < 0.001:
			self.newGesture('GST-yawn-1')




	# show all attributes
	def __repr__(self):
		string = ""
		for attr, value in sorted(self.__dict__.items()):
			if not attr.startswith('_'):
				string += str(attr) + ": " + str(value) + "\n"
		return string


	def setEmotionAttr(self, name, value, smoothing = 0):
		if smoothing == 0:
			self['name'] = value
		else:
			...		



	def newGesture(self, name, repeat = 1, speed=1, magnitude=0.9, priority=1):
		print('Creating new gesture ', name)
		
		deformObj = bpy.data.objects['deform']
		try:
			actionDatablock = bpy.data.actions[name]
		except KeyError:
			print('No gesture matching name is found')
			return

		# check value 
		checkValue(repeat, 1, 100)
		checkValue(speed, 0.1, 10)
		checkValue(magnitude, 0, 1)
		checkValue(priority, 0, 1)

		# create NLA track
		newTrack = deformObj.animation_data.nla_tracks.new()
		newTrack.name = name

		# create strip
		newStrip = newTrack.strips.new(name=name, start=1, action=actionDatablock)
		duration = (newStrip.frame_end - newStrip.frame_start)
		newStrip.blend_type = 'ADD'
		newStrip.use_animated_time = True

		if magnitude < 1:
			newStrip.use_animated_influence = True
			newStrip.influence = magnitude

		# create object and add to list
		g = Gesture(name, newTrack, newStrip, duration=duration, speed=speed, magnitude=magnitude, priority=priority, repeat=repeat)
		self.gestureList.append(g)


	def deleteGesture(self, gesture):
		print('Deleting gesture')

		# remove from list
		self.gestureList.remove(gesture)

		# remove from Blender
		deformObj = bpy.data.objects['deform']
		deformObj.animation_data.nla_tracks.remove(gesture.trackRef)


	def setEmotions(self, emotions):
		checkValue(value, 0, 1)

		bones = bpy.data.objects['control'].pose.bones
		for emotion, value in emotions.items():
			try:
				control = bones['EMO-'+emotion]
			except KeyError:
				print('Cannot set emotion. No bone with name ', emotion)
				continue
			else:
				control['intensity'] = float(value)


	def resetEmotions(self):
		bones = bpy.data.objects['control'].pose.bones
		for bone in bones:
			if bone.name.startswith('EMO-'):
				bone['intensity'] = 0.0
		

	def setPrimaryTarget(self, loc):
		self.primaryHeadTargetLoc = loc
		self._primaryHeadTargetLoc = loc

		self.primaryEyeTargetLoc = loc
		self._primaryEyeTargetLoc = loc


	def setSecondaryTarget(self, loc):
		self.secondaryHeadTargetLoc = loc
		self._secondaryHeadTargetLoc = loc
		self.secondaryEyeTargetLoc = loc
		self._secondaryEyeTargetLoc = loc
		



		
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


def init():
	if bpy:
		bpy.evaAnimationManager = AnimationManager()
	else:
		return AnimationManager()