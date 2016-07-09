# Implements the commands defined by the public API
import bpy
from mathutils import Matrix, Euler
from math import pi
from collections import OrderedDict
import logging

from rigAPI.rigAPI import RigAPI

logger = logging.getLogger('hr.blender_api.rigcontrol.commands')
# ====================================================

def init():
    bpy.ops.wm.animation_playback()
    return 0

def getEnvironment():
    return None

def terminate():
    return 0


class EvaAPI(RigAPI):
    PAU_HEAD_YAW = 1
    PAU_HEAD_PITCH = 2
    PAU_HEAD_ROLL = 4
    PAU_EYE_TARGET = 8
    PAU_FACE = 16
    # Flag which determines if currently PAU messages are being transmitted
    PAU_ACTIVE = 128
    PAU_ACTIVE_TIMEOUT = 0.5

    def __init__(self):
        # Current animation mode (combined by addition)
        # 0 - Face eyes and head controlled by animations
        # 1 - head yaw controlled by PAU
        # 2 - head pitch controlled by PAU
        # 4 - head roll controlled by PAU
        # 8 - Eye Target controlled by PAU
        # 16 - Face shapekeys controlled by PAU
        self.pauAnimationMode = 0
        # If 1 current shapekeys are controlled directly by PAU, otherwise by default drivers
        self.shapekeysControl = 0
        # Time for PAU controls to expire
        self.pauTimeout = 0
        pass


    def getAPIVersion(self):
        return 4

    def isAlive(self):
        return int(bpy.context.scene['animationPlaybackActive'])
    # Faceshift to ROS mapping functions
    def getAnimationMode(self):

        return self.pauAnimationMode

    def setAnimationMode(self, animation_mode):

        ## Now let's delete the shape
        if self.pauAnimationMode != animation_mode:
            print(animation_mode)
            # Face should drivers should be disabled
            # Face drivers are enabled on the first PAU message recieved if the correct animation mode is set.
            if animation_mode & (self.PAU_FACE | self.PAU_ACTIVE) == (self.PAU_FACE | self.PAU_ACTIVE):
                bpy.evaAnimationManager.setMode(1)
            else:
                 bpy.evaAnimationManager.setMode(0)
            self.pauAnimationMode = animation_mode
        return 0

    def setShapeKeys(self, shape_keys):
        bpy.evaAnimationManager.setShapeKeys(shape_keys)
        return 0
    # Somatic states  --------------------------------
    # awake, asleep, drunk, dazed and confused ...
    def availableSomaStates(self):
        somaStates = []
        for state in bpy.data.actions:
            if state.name.startswith("CYC-"):
                somaStates.append(state.name[4:])
        return somaStates

    def getSomaStates(self):
        eva = bpy.evaAnimationManager
        somaStates = {}
        for cycle in eva.cyclesSet:
            magnitude = round(cycle.magnitude, 3)
            rate = round(cycle.rate, 3)
            ease_in = round(cycle.ease_in, 3)
            somaStates[cycle.name] = {'magnitude': magnitude, 'rate': rate,
                'ease_in': ease_in}
        return somaStates

    def setSomaState(self, state):
        name = 'CYC-' + state['name']
        rate = state['rate']
        magnitude = state['magnitude']
        ease_in = state['ease_in']
        bpy.evaAnimationManager.setCycle(name=name,
            rate=rate, magnitude=magnitude, ease_in=ease_in)
        return 0

    # Emotion expressions ----------------------------
    # smiling, frowning, bored ...
    def availableEmotionStates(self):
        emotionStates = []
        for emo in bpy.data.objects['control'].pose.bones:
            if emo.name.startswith('EMO-'):
                emotionStates.append(emo.name[4:])
        return emotionStates


    def getEmotionStates(self):
        eva = bpy.evaAnimationManager
        emotionStates = {}
        for emotion in eva.emotionsList:
            magnitude = round(emotion.magnitude.current, 3)
            emotionStates[emotion.name] = {'magnitude': magnitude}
        return emotionStates


    def setEmotionState(self, emotion):
        # TODO: expand arguments and update doc
        bpy.evaAnimationManager.setEmotion(eval(emotion))
        return 0


    # Gestures --------------------------------------
    # blinking, nodding, shaking...
    def availableGestures(self):
        emotionGestures = []
        for gesture in bpy.data.actions:
            if gesture.name.startswith("GST-"):
                emotionGestures.append(gesture.name[4:])
        return emotionGestures


    def getGestures(self):
        eva = bpy.evaAnimationManager
        emotionGestures = {}
        for gesture in eva.gesturesList:
            duration = round(gesture.duration*gesture.repeat - gesture.stripRef.strip_time, 3)
            magnitude = round(gesture.magnitude, 3)
            speed = round(gesture.speed, 3)
            emotionGestures[gesture.name] = {'duration': duration, \
                'magnitude': magnitude, 'speed': speed}
        return emotionGestures


    def setGesture(self, name, repeat=1, speed=1, magnitude=1.0):
        bpy.evaAnimationManager.newGesture(name='GST-'+name, \
            repeat=repeat, speed=speed, magnitude=magnitude)
        return 0


    def stopGesture(self, gestureID, smoothing):
        ## TODO
        return 0

    # Visemes --------------------------------------
    def availableVisemes(self):
        visemes = []
        for viseme in bpy.data.actions:
            if viseme.name.startswith("VIS-"):
                visemes.append(viseme.name[4:])
        return visemes


    def queueViseme(self, vis, start=0, duration=0.5, \
            rampin=0.1, rampout=0.8, magnitude=1):
        return bpy.evaAnimationManager.newViseme("VIS-"+vis, duration, \
            rampin, rampout, start)

    # Eye look-at targets ==========================
    # The coordinate system used is head-relative, in 'engineering'public_ws/src/blender_api/rigControl/commands.py:135
    # coordinates: 'x' is forward, 'y' to the left, and 'z' up.
    # Distances are measured in meters.  Origin of the coordinate
    # system is somewhere (where?) in the middle of the head.

    def setFaceTarget(self, loc, speed=1.0):
        # Eva uses y==forward x==right. Distances in meters from
        # somewhere in the middle of the head.
        mloc = [loc[1], loc[0], loc[2]]
        bpy.evaAnimationManager.setFaceTarget(mloc, speed)
        return 0

    # Rotates the face target which will make head roll
    def setHeadRotation(self,rot):
        bpy.evaAnimationManager.setHeadRotation(rot)
        return 0


    def setGazeTarget(self, loc, speed=1.0):
        mloc = [loc[1],  loc[0], loc[2]]
        bpy.evaAnimationManager.setGazeTarget(mloc, speed)
        return 0
    # ========== procedural animations with unique parameters =============
    def setBlinkRandomly(self,interval_mean,interval_variation):
        bpy.evaAnimationManager.setBlinkRandomly(interval_mean,interval_variation)
        return 0

    def setSaccade(self,interval_mean,interval_variation,paint_scale,eye_size,eye_distance,mouth_width,mouth_height,weight_eyes,weight_mouth):
        bpy.evaAnimationManager.setSaccade(interval_mean,interval_variation,paint_scale,eye_size,eye_distance,mouth_width,mouth_height,weight_eyes,weight_mouth)
        return 0

    # ========== info dump for ROS, Should return non-blender data structures

    # Gets Head rotation quaternion in XYZ format in blender independamt
    # data structure.
    # Pitch: X (positive down, negative up)?
    # Yaw: Z (negative right to positive left)
    #
    # The bones['DEF-head'].id_data.matrix_world currently return the
    # unit matrix, and so are not really needed.

    def getHeadData(self):
        bones = bpy.evaAnimationManager.deformObj.pose.bones
        rhead = bones['DEF-head'].matrix * Matrix.Rotation(-pi/2, 4, 'X')
        rneck = bones['DEF-neck'].matrix * Matrix.Rotation(-pi/2, 4, 'X')
        rneck.invert()

        # I think this is the correct order for the neck rotations.
        q = (rneck * rhead).to_quaternion()
        # q = (rhead * rneck).to_quaternion()
        return {'x':q.x, 'y':q.y, 'z':q.z, 'w':q.w}

    # Same as head, but for the lower neck joint.
    def getNeckData(self):
        bones = bpy.evaAnimationManager.deformObj.pose.bones
        rneck = bones['DEF-neck'].matrix * Matrix.Rotation(-pi/2, 4, 'X')
        q = rneck.to_quaternion()
        return {'x':q.x, 'y':q.y, 'z':q.z, 'w':q.w}

    # Gets Eye rotation angles:
    # Pitch: down(negative) to up(positive)
    # Yaw: left (negative) to right(positive)

    def getEyesData(self):
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


    def getFaceData(self):
        shapekeys = OrderedDict()
        for shapekeyGroup in bpy.data.shape_keys:
            # Hardcoded to find the correct group
            if shapekeyGroup.name == 'ShapeKeys':
                for kb in shapekeyGroup.key_blocks:
                    shapekeys[kb.name] = kb.value

        # Fake the jaw shapekey from its z coordinate
        jawz = bpy.evaAnimationManager.deformObj.pose.bones['chin'].location[2]
        shapekeys['jaw'] = min(max(jawz*7.142, 0), 1)

        return shapekeys


    def setNeckRotation(self, pitch, roll):
        bpy.evaAnimationManager.deformObj.pose.bones['DEF-neck'].rotation_euler = Euler((pitch, 0, roll))

    def setParam(self, key, value):
        cmd = "%s=%s" % (str(key), str(value))
        logger.info("Run %s" % cmd)
        try:
            exec(cmd)
        except Exception as ex:
            logger.error("Error %s" % ex)
            return False
        return True

    def getParam(self, param):
        param = param.strip()
        logger.info("Get %s" % param)
        try:
            return str(eval(param))
        except Exception as ex:
            logger.error("Error %s" % ex)

    def getAnimationLength(self, animation):
        animation = "GST-"+animation
        if not animation in bpy.data.actions.keys():
            return 0
        else:
            frame_range = bpy.data.actions[animation].frame_range
            frames = 1+frame_range[1]-frame_range[0]
            return frames / bpy.context.scene.render.fps

