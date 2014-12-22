# Implements the commands defined by the public API
import bpy
from  mathutils import Matrix
from math import pi
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


def setGesture(name, repeat=1, speed=1, magnitude=0.5):
	bpy.evaAnimationManager.newGesture(name, \
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


# ========== info dump for ROS, Should return non-blender data structures

# Gets Head rotation quaternion in XYZ formot in blender independamt data structure.
# Pitch: X (positive down, negative up)?
# Yaw: Z (negative right to positive left)

def getHeadData():
	bones = bpy.evaAnimationManager.deformObj.pose.bones
	q = (bones['DEF-head'].id_data.matrix_world*bones['DEF-head'].matrix*Matrix.Rotation(-pi/2, 4, 'X')).to_quaternion()
	return {'x':q.x, 'y':q.y, 'z':q.z, 'w':q.w}

# Gets Eye rotation angles:
# Pitch: down(negative) to up(positive)
# Yaw: left (negative) to right(positive)

def getEyesData():
	bones = bpy.evaAnimationManager.deformObj.pose.bones
	leye = bones['eye.L'].matrix.to_euler()
	reye = bones['eye.R'].matrix.to_euler()
	leye_p = leye.x
	leye_y = pi - leye.z if leye.z >= 0 else -(pi+leye.z)
	reye_p = reye.x
	reye_y = pi - reye.z if reye.z >= 0 else -(pi+reye.z)
	return {'l':{'p':leye_p,'y':leye_y},'r':{'p':reye_p,'y':reye_y}}


def getFaceData():
	shapekeys = {}
	for shapekeyGroup in bpy.data.shape_keys:
		if shapekeyGroup.name == 'Key.007':          # hardcoded to find the correct group
			for kb in shapekeyGroup.key_blocks:
				shapekeys[kb.name] = kb.value
	return shapekeys
