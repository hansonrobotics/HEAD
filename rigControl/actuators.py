from .helpers import *

import random


def idleCycle(self):
	for loopBone in bpy.data.objects['control'].pose.bones:
		if "CYC" in loopBone.name: 
			if loopBone.location[1] < 1:
				loopBone.location[1] += 1/(4*loopBone['frames'])
			else :
				loopBone.location[1] = 0


	bpy.data.objects['control'].pose.bones["CYC-normal"]["influence"] = 1.0


def breathingCycle(self):
	if self.checkElapsed('breathing', 600):
		self.newGesture('CYC-breathing', repeat=60, speed=1, magnitude=0.2)
		self.setElapsed('breathing')


def eyeSaccades(self):
	''' applies random saccades to eye '''
	newLoc = [0,0,0]
	newLoc[0] = random.gauss(self.primaryEyeTargetLoc.current[0], 0.05)
	newLoc[1] = random.gauss(self.primaryEyeTargetLoc.current[1], 0.02)
	newLoc[2] = random.gauss(self.primaryEyeTargetLoc.current[2], 0.02)

	# compute distance from previous eye position
	distance = computeDistance(newLoc, self.primaryEyeTargetLoc.current)

	if distance > 0.1:
		if self.randomFrequency('blinkMicro', 1.0):
			self.newGesture('GST-blink-micro')
				
	# override eye movement
	self.primaryEyeTargetLoc.current = newLoc
