# Functions that control autonomous behaviors in the Eva character
# Breathing, blinking, eye saccades, head drift.
#
# All of the functions here take AnimationManager as the argument.

from .helpers import *

import random


def idleCycle(self):
	if 'CYC-normal' not in [gesture.name for gesture in self.gesturesList]:
		self.newGesture('CYC-normal', repeat = 10, speed=1, magnitude=1.0, priority=1)


def breathingCycle(self, rate, intensity):
	if 'CYC-breathing' not in [gesture.name for gesture in self.gesturesList]:
		# create new strip
		self.newGesture('CYC-breathing', repeat=10, speed=rate, magnitude=intensity)
	else:
		# update strip property
		for gesture in self.gesturesList:
			if gesture.name == 'CYC-breathing':
				gesture.stripRef.influence = intensity
				gesture.speed = rate


def emotionJitter(self):
	for emotion in self.emotionsList:
		target = emotion.magnitude.target
		emotion.magnitude.target = random.gauss(target, target/20)


def eyeSaccades(self, eyeWanderAbs):
	''' applies random saccades to eye '''
	newLoc = [0,0,0]
	newLoc[0] = random.gauss(self.eyeTargetLoc.current[0], eyeWanderAbs)
	newLoc[1] = self.eyeTargetLoc.current[1]
	newLoc[2] = random.gauss(self.eyeTargetLoc.current[2], eyeWanderAbs * 0.5)

	# compute distance from previous eye position
	distance = computeDistance(newLoc, self.eyeTargetLoc.current)

	if distance > 0.1:
		if self.randomFrequency('blinkMicro', 1.0):
			self.newGesture('GST-blink-micro')
				
	# override eye movement
	self.eyeTargetLoc.current = newLoc


def headDrift(self):
	''' applies random head drift '''
	loc = [0,0,0]
	loc[0] = random.gauss(self.headTargetLoc.target[0], self.headTargetLoc.target[0]/100)
	loc[1] = random.gauss(self.headTargetLoc.target[1], self.headTargetLoc.target[1]/100)
	loc[2] = random.gauss(self.headTargetLoc.target[2], self.headTargetLoc.target[2]/100)
	self.headTargetLoc.target = loc


def blink(self, duration):
	# compute probability
	micro = -(abs(duration+1)-1)
	normal = -(abs(duration+0)-1)
	relaxed = -(abs(duration-1)-1)
	sleepy = -(abs(duration-2)-1)

	micro = max(micro, 0)
	normal = max(normal, 0)
	relaxed = max(relaxed, 0)
	sleepy = max(sleepy, 0)

	index = randomSelect([micro, normal, relaxed, sleepy])
	action = ['GST-blink-micro', 'GST-blink', 'GST-blink-relaxed', 'GST-blink-sleepy']
	self.newGesture(action[index])
	
	# print(action[index])
