# Implements the commands defined by the public API
import bpy
from  mathutils import Matrix
from math import pi
from collections import OrderedDict
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
	bpy.evaAnimationManager.newGesture(name='GST-'+name, \
		repeat=repeat, speed=speed, magnitude=magnitude)
	return 0

# The coordinate system used is head-relative, in 'engineering'
# coordinates: 'x' is forward, 'y' to the left, and 'z' up.
# Distances are measured in meters.  Origin of the coordinate
# system is somewhere (where?) in the middle of the head.

def setFaceTarget(loc):
	# Eva uses y==forward x==right. Distances in meters from
	# somewhere in the middle of the head.
	mloc = [-loc[1], loc[0], loc[2]]
	bpy.evaAnimationManager.setFaceTarget(mloc)
	return 0

def setGazeTarget(loc):
	mloc = [-loc[1],  loc[0], loc[2]]
	bpy.evaAnimationManager.setGazeTarget(mloc)
	return 0

def getGestureParams():
	eva = bpy.evaAnimationManager
	return {'eyeDartRate': round(eva.eyeDartRate, 3),
			'eyeWander': round(eva.eyeWander, 3),
			'blinkRate': round(eva.blinkRate, 3),
			'blinkDuration': round(eva.blinkDuration, 3),
			'breathRate': round(eva.breathRate, 3),
			'breathIntensity': round(eva.breathIntensity, 3)}

def setVisemes(vis):
	return bpy.evaAnimationManager.newViseme(vis)

# ========== info dump for ROS, Should return non-blender data structures

# Gets Head rotation quaternion in XYZ format in blender independamt
# data structure.
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
	head = (bones['DEF-head'].id_data.matrix_world*bones['DEF-head'].matrix*Matrix.Rotation(-pi/2, 4, 'X')).to_euler()
	leye = bones['eye.L'].matrix.to_euler()
	reye = bones['eye.R'].matrix.to_euler()
	# Relative to head. Head angles are inversed.
	leye_p = leye.x + head.x
	leye_y = pi - leye.z if leye.z >= 0 else -(pi+leye.z)
	reye_p = reye.x + head.x
	reye_y = pi - reye.z if reye.z >= 0 else -(pi+reye.z)
	# Add head target
	leye_y += head.z
	reye_y += head.z
	return {'l':{'p':leye_p,'y':leye_y},'r':{'p':reye_p,'y':reye_y}}


def getFaceData():
	shapekeys = OrderedDict()
	for shapekeyGroup in bpy.data.shape_keys:
		# Hardcoded to find the correct group
		if shapekeyGroup.name == 'Key.007':
			for kb in shapekeyGroup.key_blocks:
				shapekeys[kb.name] = kb.value

	# Fake the jaw shapekey from its z coordinate
	jawz = bpy.evaAnimationManager.deformObj.pose.bones['chin'].location[2]
	shapekeys['jaw'] = min(max(jawz/0.3, 0), 1)

	return shapekeys
