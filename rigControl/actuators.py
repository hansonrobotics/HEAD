from .helpers import *

import random


def idleCycle(self):
	if 'CYC-normal' not in [gesture.name for gesture in self.gesturesList]:
		self.newGesture('CYC-normal', repeat = 10, speed=1, magnitude=1.0, priority=1)


def breathingCycle(self):
	if 'CYC-breathing' not in [gesture.name for gesture in self.gesturesList]:
		self.newGesture('CYC-breathing', repeat=10, speed=1, magnitude=1.0)


def eyeSaccades(self):
	''' applies random saccades to eye '''
	newLoc = [0,0,0]
	newLoc[0] = random.gauss(self.primaryEyeTargetLoc.current[0], 0.05)
	newLoc[1] = random.gauss(self.primaryEyeTargetLoc.current[1], 0.03)
	newLoc[2] = random.gauss(self.primaryEyeTargetLoc.current[2], 0.03)

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
	loc[0] = random.gauss(self.primaryHeadTargetLoc.target[0], 0.01)
	loc[1] = random.gauss(self.primaryHeadTargetLoc.target[1], 0.01)
	loc[2] = random.gauss(self.primaryHeadTargetLoc.target[2], 0.01)
	self.primaryHeadTargetLoc.target = loc
