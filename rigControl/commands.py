import bpy

def getAPIVersion():
	return 1

def init():
	...

def getEnvironment():
	...

def isAlive():
	...

def terminate():
	...

def availableEmotionStates():
	emotionStates = []
	for emo in bpy.data.objects['control'].pose.bones:
		if "EMO" in emo.name:
			emotionStates.append(emo.name)
	return emotionStates


def getEmotionStates(animationManager):
	emotionStates = {}
	for name, emotion in animationManager.emotions.items():
		emotionStates[name] = round(emotion.current, 3)
	return emotionStates


def setEmotionStates():
	pass


def availableEmotionGestures():
	emotionGestures = []
	for gesture in bpy.data.actions:
		if "GST" in gesture.name:
			emotionGestures.append(gesture.name)
	return emotionGestures


def getEmotionGestures(animationManager):
	emotionGestures = []
	for gesture in animationManager.gestureList:
		data = [gesture.name, gesture.duration*gesture.repeat - gesture.stripRef.strip_time]
		emotionGestures.append(data)
	return emotionGestures


def setEmotionGestures():
	pass


def setPrimaryTarget():
	pass
	

def setSecondaryTarget():
	pass