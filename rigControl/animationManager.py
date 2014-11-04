import bpy

class AnimationManager():
	
	def __init__(self):
		self.gestureList = []
		print('Animation Manager singleton started')

	def newGesture(self, name, track, strip):
		g = Gesture(name, track, strip)
		self.gestureList.append(g)


	def deleteGesture(self, gesture):
		self.gestureList.remove(gesture)

		deformObj = bpy.data.objects['deform']
		deformObj.animation_data.nla_tracks.remove(gesture.trackRef)


class Gesture():
	
	def __init__(self, name, track, strip):
		self.name = name
		self.duration = 200.0
		self.magnitude = 1
		self.speed = 1
		self.weight = 1

		self.trackRef = track
		self.stripRef = strip
		self.stripTime = 1.0


def init():
	bpy.evaAnimationManager = AnimationManager()