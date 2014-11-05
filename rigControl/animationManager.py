import bpy

class AnimationManager():
	
	def __init__(self):
		print('Starting AnimationManager singleton')
		self.gestureList = []

	def newGesture(self, name):
		print('Creating new gesture ', name)
		
		deformObj = bpy.data.objects['deform']
		actionDatablock = bpy.data.actions[name]

		# create NLA track
		newTrack = deformObj.animation_data.nla_tracks.new()
		newTrack.name = name

		# create strip
		actionName = name
		action = actionDatablock
		curFrame = 1
		newStrip = newTrack.strips.new(name=actionName, start=curFrame, action=action)
		newStrip.blend_type = 'ADD'
		newStrip.use_animated_time = True


		duration = newStrip.frame_end - newStrip.frame_start
		speed = 1
		magnitude = 1
		priority = 1
		repeat = 1

		# set strip prop
		newStrip.scale = 1.0/max(speed, 0.01)
		newStrip.repeat = repeat
		
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


		
class Gesture():
	
	def __init__(self, name, track, strip, duration=100, speed=1, magnitude=1, priority=1, repeat=1):
		self.name = name
		self.duration = duration
		self.magnitude = magnitude
		self.speed = speed
		self.priority = priority
		repeat = repeat

		self.trackRef = track
		self.stripRef = strip
		self.stripTime = 1.0


def init():
	bpy.evaAnimationManager = AnimationManager()