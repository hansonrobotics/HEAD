import bpy

def getAPIVersion():
	return 1

def init():
	bpy.ops.wm.animation_playback()

def getEnvironment():
	...

def isAlive():
	return bpy.context.scene['animationPlaybackActive']

def terminate():
	...

def availableEmotionStates():
	emotionStates = []
	for emo in bpy.data.objects['control'].pose.bones:
		if emo.name.startswith('EMO-'):
			emotionStates.append(emo.name[4:])
	return emotionStates


def getEmotionStates():
	eva = bpy.evaAnimationManager
	emotionStates = {}
	for emotion in eva.emotionsList:
		magnitude = round(emotion.magnitude.current, 3)
		duration = round(emotion.duration, 3)
		emotionStates[emotion.name] = {'magnitude': magnitude, 'duration': duration}
	return emotionStates


def setEmotionStates(emotions):
	bpy.evaAnimationManager.setEmotions(emotions)


def availableEmotionGestures():
	emotionGestures = []
	for gesture in bpy.data.actions:
		if gesture.name.startswith("GST-"):
			emotionGestures.append(gesture.name[4:])
	return emotionGestures


def getEmotionGestures():
	eva = bpy.evaAnimationManager
	emotionGestures = {}
	for gesture in eva.gesturesList:
		duration = round(gesture.duration*gesture.repeat - gesture.stripRef.strip_time, 3)
		magnitude = round(gesture.magnitude, 3)
		speed = round(gesture.speed, 3)
		emotionGestures[gesture.name] = {'duration': duration, 'magnitude': magnitude, 'speed': speed}
	return emotionGestures


def setEmotionGestures(name):
	bpy.evaAnimationManager.newGesture(name=name)
	


def setPrimaryTarget():
	pass


def setSecondaryTarget():
	pass



def getEmotionParams():
	eva = bpy.evaAnimationManager
	return {'eyeDartRate': round(eva.eyeDartRate, 3),
			'eyeWander': round(eva.eyeWander, 3),
			'blinkRate': round(eva.blinkRate, 3),
			'blinkDuration': round(eva.blinkDuration, 3),
			'breathRate': round(eva.breathRate, 3),
			'breathIntensity': round(eva.breathIntensity, 3)}
