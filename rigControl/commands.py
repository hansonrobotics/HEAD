# Implements the commands defined by the public API

import bpy

# System control and information commands ===========
def getAPIVersion():
	return 1

def init():
	bpy.ops.wm.animation_playback()
	return 0

def getEnvironment():
	...
	return None

def isAlive():
	return bpy.context.scene['animationPlaybackActive']

def terminate():
	...
	return 0

# Emotion and Gesture commands ==========================

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


def setEmotionState(emotion):
	bpy.evaAnimationManager.setEmotion(eval(emotion))
	return 0


# Gestures --------------------------------------
def availableGestures():
	emotionGestures = []
	for gesture in bpy.data.actions:
		if gesture.name.startswith("GST-"):
			emotionGestures.append(gesture.name[4:])
	return emotionGestures


def getGestures():
	eva = bpy.evaAnimationManager
	emotionGestures = {}
	for gesture in eva.gesturesList:
		duration = round(gesture.duration*gesture.repeat - gesture.stripRef.strip_time, 3)
		magnitude = round(gesture.magnitude, 3)
		speed = round(gesture.speed, 3)
		emotionGestures[gesture.name] = {'duration': duration, \
			'magnitude': magnitude, 'speed': speed}
	return emotionGestures


def setGesture(name, repeat, speed, magnitude):
	bpy.evaAnimationManager.newGesture(name='GST-'+name, \
		repeat=repeat, speed=speed, magnitude=magnitude)
	return 0

def setPrimaryTarget(loc):
	bpy.evaAnimationManager.setPrimaryTarget(loc)
	return 0

def setSecondaryTarget(loc):
	bpy.evaAnimationManager.setSecondaryTarget(loc)
	return 0

def engageTarget():
	return 0


def getGestureParams():
	eva = bpy.evaAnimationManager
	return {'eyeDartRate': round(eva.eyeDartRate, 3),
			'eyeWander': round(eva.eyeWander, 3),
			'blinkRate': round(eva.blinkRate, 3),
			'blinkDuration': round(eva.blinkDuration, 3),
			'breathRate': round(eva.breathRate, 3),
			'breathIntensity': round(eva.breathIntensity, 3)}


# ========== info dump for ROS, to be invoked automatically with Blender in the future
def getHeadData():
	bones = bpy.evaAnimationManager.deformObj.pose.bones
	return bones['head'].matrix


def getEyesData():
	bones = bpy.evaAnimationManager.deformObj.pose.bones
	both = [bones['eye.L'].matrix.to_euler(), bones['eye.R'].matrix.to_euler()]
	return both


def getFaceData():
	shapekeys = {}
	for shapekeyGroup in bpy.data.shape_keys:
		if shapekeyGroup.name == 'Key.007':          # hardcoded to find the correct group
			for kb in shapekeyGroup.key_blocks:
				shapekeys[kb.name] = kb.value
	return shapekeys
