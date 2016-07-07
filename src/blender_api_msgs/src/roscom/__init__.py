# Implements ROS node to convert from ROS messages to the
# blender API defined in rigAPI.py.  The actual commands
# are transmitted to blender using the CommandListener.
#
import queue

import rospy
import blender_api_msgs.msg as msg
import blender_api_msgs.srv as srv
import pau2motors.msg as paumsg
import std_msgs.msg as stdmsg
import geometry_msgs.msg as geomsg
import logging
import math
from mathutils import *
import time

import os
run_id = rospy.get_param('/run_id', None)
if run_id and 'ROS_LOG_DIR' in os.environ:
    os.environ['ROS_LOG_DIR'] = os.path.join(
        os.path.expanduser(os.environ['ROS_LOG_DIR']), run_id)

api = None
logger = logging.getLogger('hr.blender_api_msgs.roscom')

# This is called by __init__.py
# rigapi should be a (non-abstract-base-class) instance of RigAPI
def build(rigapi):
    global api
    api = rigapi
    return RosNode()

# RosNode duck types the virtual class rigAPI.CommandSource that blender_api
# expects us to use as the API to the rig.
class RosNode():
    '''All of class state is stored in self.incoming_queue and self.topics '''

    def __init__(self):
        self.incoming_queue = queue.Queue()
        rospy.init_node('blender_api')

        # Collect all public methods in CommandWrappers class.
        # Note that, because of the applied decorators, the methods are actually
        # CommandDecorator objects.
        self.topics = [
            getattr(CommandWrappers, name) for name in dir(CommandWrappers)
            if name[0] != '_'
        ]
        # Advertise the publishers and subscribers.
        for topic in self.topics:
            # Direct '@subscribe' decorated topics to our queue.
            if isinstance(topic, subscribe):
                topic.callback = self._enqueue
            topic.register()

    def init(self):
        for topic in self.topics:
            topic.paused = False
        return True

    def poll(self):
        '''Incoming cmd getter '''
        try:
            return self.incoming_queue.get_nowait()
        except queue.Empty:
            return None

    def push(self):
        '''Create and publish messages to '@publish_live' decorated topics '''
        live_topics = [topic for topic in self.topics if isinstance(topic, publish_live)]
        try:
            for topic in live_topics:
                topic.publish()
        except rospy.ROSException as ex:
            logger.error(ex)
            return False
        return True

    # After this is called, blender will not ever poll us again.
    def drop(self):
        '''Disable communication'''
        # We don't shutdown the actual ROS node, because restarting a
        # ROS node after shutdown is not supported in rospy.
        for topic in self.topics:
            topic.drop()
        return True

    def _enqueue(self, incoming_cmd):
        self.incoming_queue.put(incoming_cmd)


class IncomingCmd:
    ''' a function (command) prepared for delayed execution '''
    def __init__(self, func, arg):
        self.func, self.arg = func, arg
    def execute(self):
        self.func(self.arg)


# Decorators for CommandWrappers methods

class CommandDecorator:
    def __init__(self, topic, dataType):
        self.topic = topic
        self.dataType = dataType
        self.paused = False
    def __call__(self, cmd_func):
        self.cmd_func = cmd_func
        return self
    def register(self): raise NotImplementedError
    def drop(self):
        self.paused = True

class publish_once(CommandDecorator):
    def register(self):
        self.pub = rospy.Publisher(self.topic, self.dataType, queue_size=1, latch=True)
        self.pub.publish(self.cmd_func())

class publish_live(CommandDecorator):
    def register(self):
        self.pub = rospy.Publisher(self.topic, self.dataType, queue_size=1)
    def publish(self):
        if self.paused: return
        self.pub.publish(self.cmd_func())

class subscribe(CommandDecorator):
    def register(self):
        self.sub = rospy.Subscriber(self.topic, self.dataType, self._handle)
    def _handle(self, msg):
        # XXX ??? Now that ROS is not initialized twice, the
        # self.callback thing is not working.  I cannot tell what
        # it was supposed to do ???
        # self.callback(IncomingCmd(self.cmd_func, msg))
        if self.paused: return
        self.cmd_func(msg)

class service(CommandDecorator):
    def register(self):
        self.serv = rospy.Service(self.topic, self.dataType, self._handle)
    def _handle(self, msg):
        return self.cmd_func(msg)

class CommandWrappers:
    '''
    These methods shouldn't be called directly.
    They define topics the rosnode will publish and subscribe to.

    These methods must be decorated with a CommandDecorator subclass.
    The decorators take two arguments: a topicname and a message type they are
    meant to send or receive.
    '''

    @publish_once("~get_api_version", msg.GetAPIVersion)
    def getAPIVersion():
        return msg.GetAPIVersion(api.getAPIVersion())
    # ROS interfaces to listen to blendshape topics

    @publish_live("~get_animation_mode", stdmsg.UInt8)
    def getAnimationMode():
        return stdmsg.UInt8(api.getAnimationMode())

    @subscribe("~set_animation_mode", stdmsg.UInt8)
    def setAnimationMode(mesg):
        mode= mesg.data
        api.setAnimationMode(mode)

    # Somatic states  --------------------------------
    # awake, asleep, breathing, drunk, dazed and confused ...
    @publish_once("~available_soma_states", msg.AvailableSomaStates)
    def availableSomaStates():
        return msg.AvailableSomaStates(api.availableSomaStates())


    @subscribe("~set_soma_state", msg.SomaState)
    def setSomaState(mesg):
        if api.pauAnimationMode & api.PAU_ACTIVE:
            return
        state = {
            'name': mesg.name,
            'magnitude': mesg.magnitude,
            'rate': mesg.rate,
            'ease_in': mesg.ease_in.to_sec()
        }
        api.setSomaState(state)

    @publish_live("~get_soma_states", msg.SomaStates)
    def getSomaStates():
        return msg.SomaStates([
            msg.SomaState(name,
                vals['magnitude'],
                vals['rate'],
                rospy.Duration(vals['ease_in']))
            for name, vals in api.getSomaStates().items()
        ])


    # Emotion expressions ----------------------------
    # smiling, frowning, bored ...
    @publish_once("~available_emotion_states", msg.AvailableEmotionStates)
    def availableEmotionStates():
        return msg.AvailableEmotionStates(api.availableEmotionStates())


    @publish_live("~get_emotion_states", msg.EmotionStates)
    def getEmotionStates():
        return msg.EmotionStates([
            # Emotion state has no current duration
            msg.EmotionState(name, vals['magnitude'], 0)
            for name, vals in api.getEmotionStates().items()
        ])

    # Message is a single emotion state
    @subscribe("~set_emotion_state", msg.EmotionState)
    def setEmotionState(mesg):
        if api.pauAnimationMode & (api.PAU_ACTIVE > api.PAU_FACE) == api.PAU_ACTIVE | api.PAU_FACE:
            return
        emotion = str({
            mesg.name: {
                'magnitude': mesg.magnitude,
                'duration': mesg.duration.to_sec()
            }
        })
        api.setEmotionState(emotion)


    # Gestures --------------------------------------
    # blinking, nodding, shaking...
    @publish_once("~available_gestures", msg.AvailableGestures)
    def availableGestures():
        return msg.AvailableGestures(api.availableGestures())


    @publish_live("~get_gestures", msg.Gestures)
    def getGestures():
        return msg.Gestures([
            msg.Gesture(
                name,
                vals['speed'],
                vals['magnitude'],
                rospy.Duration(vals['duration'])
            ) for name, vals in api.getGestures().items()
        ])


    @subscribe("~set_gesture", msg.SetGesture)
    def setGesture(msg):
        if api.pauAnimationMode & api.PAU_ACTIVE > 0:
            return
        try:
            api.setGesture(msg.name, msg.repeat, msg.speed, msg.magnitude)
        except TypeError:
            logger.error('Unknown gesture: {}'.format(msg.name))


    # Visemes --------------------------------------
    @publish_once("~available_visemes", msg.AvailableVisemes)
    def availableVisemes():
        return msg.AvailableVisemes(api.availableVisemes())

    @subscribe("~queue_viseme", msg.Viseme)
    def queueViseme(msg):
        if api.pauAnimationMode & (api.PAU_ACTIVE | api.PAU_FACE) == api.PAU_ACTIVE | api.PAU_FACE:
            return
        try:
            api.queueViseme(msg.name, msg.start.to_sec(),
                msg.duration.to_sec(),
                msg.rampin, msg.rampout, msg.magnitude)
        except TypeError:
            logger.error('Unknown viseme: {}'.format(msg.name))


    # Look-at and turn-to-face targets ---------------------
    # Location that Eva will look at and face.
    @subscribe("~set_face_target", msg.Target)
    def setFaceTarget(msg):
        if api.pauAnimationMode & (api.PAU_ACTIVE | api.PAU_HEAD_YAW) == api.PAU_ACTIVE | api.PAU_HEAD_YAW:
            return
        flist = [msg.x, msg.y, msg.z]
        api.setFaceTarget(flist, msg.speed)

    # Location that Eva will look at (only).
    @subscribe("~set_gaze_target", msg.Target)
    def setGazeTarget(msg):
        if api.pauAnimationMode & (api.PAU_ACTIVE | api.PAU_EYE_TARGET) == api.PAU_ACTIVE | api.PAU_EYE_TARGET:
            return
        flist = [msg.x, msg.y, msg.z]
        api.setGazeTarget(flist, msg.speed)

    @subscribe("~set_head_rotation", stdmsg.Float32)
    def setHeadRotation(msg):
        if api.pauAnimationMode & (api.PAU_ACTIVE | api.PAU_HEAD_ROLL) == api.PAU_ACTIVE | api.PAU_HEAD_ROLL:
            return
        #sets only pitch and roll
        api.setHeadRotation(msg.data)

    # Pau messages --------------------------------
    @publish_live("~get_pau", paumsg.pau)
    def getPau():
        msg = paumsg.pau()

        head = api.getHeadData()
        msg.m_headRotation.x = head['x']
        msg.m_headRotation.y = head['y']
        msg.m_headRotation.z = -head['z']
        msg.m_headRotation.w = head['w']

        neck = api.getNeckData()
        msg.m_neckRotation.x = neck['x']
        msg.m_neckRotation.y = neck['y']
        msg.m_neckRotation.z = -neck['z']
        msg.m_neckRotation.w = neck['w']

        eyes = api.getEyesData()
        msg.m_eyeGazeLeftPitch = eyes['l']['p']
        msg.m_eyeGazeLeftYaw = eyes['l']['y']
        msg.m_eyeGazeRightPitch = eyes['r']['p']
        msg.m_eyeGazeRightYaw = eyes['r']['y']
        shapekeys = api.getFaceData()

        msg.m_coeffs = shapekeys.values()
        # Manage timeout for set_pau
        if api.pauTimeout < time.time():
            api.setAnimationMode(api.pauAnimationMode & ~api.PAU_ACTIVE)
        return msg

    # Set Pau messages -----------------------------
    @subscribe("~set_pau", paumsg.pau)
    def setPau(msg):
        # Ignore if no animations are enabled by PAU
        if api.pauAnimationMode == 0:
            return
        # Active mode expires
        api.pauTimeout = time.time()+api.PAU_ACTIVE_TIMEOUT
        # PAU animation is active
        api.setAnimationMode(api.pauAnimationMode | api.PAU_ACTIVE)


        # Calculate head and eyes targets
        pitch = 0
        yaw = 0
        roll = 0
        if api.pauAnimationMode & (api.PAU_HEAD_YAW | api.PAU_HEAD_ROLL):
            q = msg.m_headRotation
            q = Quaternion([q.w,q.x,q.y,q.z])
            try:
                e = q.to_euler('XZY')
                pitch = e.x
                yaw = e.y
                roll = e.z
            except:
                pitch = 0
                yaw = 0
                roll = 0
            az = math.sin(pitch)
            ay = math.sin(yaw)*math.cos(pitch)
            # Target one meter away
            ax = math.cos(yaw)*math.cos(pitch)
            # Sets Face target
            if api.pauAnimationMode & api.PAU_HEAD_YAW:
                api.setFaceTarget([ax, ay, -az])
            if api.pauAnimationMode & api.PAU_HEAD_ROLL:
                api.setHeadRotation(roll)

        if api.pauAnimationMode & api.PAU_EYE_TARGET:
            pitch += math.radians(msg.m_eyeGazeLeftPitch)
            yaw += math.radians(msg.m_eyeGazeLeftYaw)
            az = math.sin(pitch)
            ay = math.sin(yaw)*math.cos(pitch)
            # Target one meter away
            ax = math.cos(yaw)*math.cos(pitch)
            # Sets Face target
            api.setGazeTarget([ax, ay, -az])
        if api.pauAnimationMode & api.PAU_FACE:
            # Set Face shapekeys
            shapekeys = dict(zip(msg.m_shapekeys, msg.m_coeffs))
            api.setShapeKeys(shapekeys)

    @subscribe("~set_neck_rotation", geomsg.Vector3)
    def setNeckRotation(msg):
        #sets only pitch and roll
        api.setNeckRotation(msg.y, msg.x)

    @subscribe("~set_blink_randomly",msg.BlinkCycle)
    def setBlinkRandomly(msg):
        api.setBlinkRandomly(msg.mean,msg.variation)

    @subscribe("~set_saccade",msg.SaccadeCycle)
    def setSaccade(msg):
        api.setSaccade(msg.mean,msg.variation,msg.paint_scale,msg.eye_size,msg.eye_distance,msg.mouth_width,msg.mouth_height,msg.weight_eyes,msg.weight_mouth)

    @service("~set_param", srv.SetParam)
    def setParam(msg):
        return srv.SetParamResponse(api.setParam(msg.key, msg.value))

    @service("~get_param", srv.GetParam)
    def getParam(msg):
        return srv.GetParamResponse(api.getParam(msg.param))

    @service("~get_animation_length", srv.GetAnimationLength)
    def getAnimationLength(req):
        return srv.GetAnimationLengthResponse(api.getAnimationLength(req.animation))

