import bpy

class AnimationManager():
	
	def __init__(self):
		print('Starting AnimationManager singleton')
		self.gestureList = []
		self.primaryTargetLoc = [0,0,0]

		# create keep alive track
		# self.newGesture('CYC-normal', repeat = 100)

	def newGesture(self, name, repeat = 1):
		print('Creating new gesture ', name)
		
		deformObj = bpy.data.objects['deform']
		try:
			actionDatablock = bpy.data.actions[name]
		except KeyError:
			print('No gesture matching name is found')
			return


		# create NLA track
		newTrack = deformObj.animation_data.nla_tracks.new()
		newTrack.name = name

		speed = 1
		magnitude = 1
		priority = 1

		# create strip
		newStrip = newTrack.strips.new(name=name, start=1, action=actionDatablock)
		duration = newStrip.frame_end - newStrip.frame_start
		newStrip.blend_type = 'ADD'
		newStrip.use_animated_time = True
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


	def setEmotions(self, emotions):
		clampMax = 1.0
		clampMin = 0.0
		bones = bpy.data.objects['control'].pose.bones
		for emotion, value in emotions.items():
			try:
				control = bones['EMO-'+emotion]
			except KeyError:
				print('No bone with name ', emotion)
				continue
			else:
				control['intensity'] = max(min(float(value), clampMax),clampMin)


	def resetEmotions(self):
		bones = bpy.data.objects['control'].pose.bones
		for bone in bones:
			if bone.name.startswith('EMO-'):
				bone['intensity'] = 0.0
		

	def setPrimaryTarget(self, loc):
		self.primaryTargetLoc = loc

		



		
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