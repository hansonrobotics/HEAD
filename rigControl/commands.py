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
		if emo.name.startswith('EMO-'):
			emotionStates.append(emo.name[4:])
	return emotionStates


def getEmotionStates(animationManager):
	emotionStates = {}
	for name, emotion in animationManager.emotions.items():
		emotionStates[name] = round(emotion.current, 3)
	return emotionStates


def setEmotionStates(emotions, animationManager):
		animationManager.setEmotions(emotions)


def availableEmotionGestures():
	emotionGestures = []
	for gesture in bpy.data.actions:
		if gesture.name.startswith("GST-"):
			emotionGestures.append(gesture.name[4:])
	return emotionGestures


def getEmotionGestures(animationManager):
	emotionGestures = []
	for gesture in animationManager.gesturesList:
		data = [gesture.name, gesture.duration*gesture.repeat - gesture.stripRef.strip_time]
		emotionGestures.append(data)
	return emotionGestures


def setEmotionGestures():
	pass


def setPrimaryTarget():
	pass


def setSecondaryTarget():
	pass