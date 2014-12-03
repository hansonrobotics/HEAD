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
		emotion.magnitude.target = random.gauss(target, target/10)


def eyeSaccades(self, eyeWanderAbs):
	''' applies random saccades to eye '''
	newLoc = [0,0,0]
	newLoc[0] = random.gauss(self.primaryEyeTargetLoc.current[0], eyeWanderAbs)
	newLoc[1] = self.primaryEyeTargetLoc.current[1]
	newLoc[2] = random.gauss(self.primaryEyeTargetLoc.current[2], eyeWanderAbs * 0.5)

	# compute distance from previous eye position
	distance = computeDistance(newLoc, self.primaryEyeTargetLoc.current)

	if distance > 0.1:
		if self.randomFrequency('blinkMicro', 1.0):
			self.newGesture('GST-blink-micro')
				
	# override eye movement
	self.primaryEyeTargetLoc.current = newLoc


def headDrift(self):
	''' applies random head drift '''
	loc = [0,0,0]
	loc[0] = random.gauss(self.primaryHeadTargetLoc.target[0], 0.005)
	loc[1] = random.gauss(self.primaryHeadTargetLoc.target[1], 0.005)
	loc[2] = random.gauss(self.primaryHeadTargetLoc.target[2], 0.005)
	self.primaryHeadTargetLoc.target = loc


def blink(self, duration):
	if duration < -0.5:
		self.newGesture('GST-blink-micro')
	elif duration < 0.5:
		self.newGesture('GST-blink')
	elif duration < 1.5:
		self.newGesture('GST-blink-relaxed')
	elif duration < 2.5:
		self.newGesture('GST-blink-sleepy')
