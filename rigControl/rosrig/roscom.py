# Implements ROS node to convert from ROS messages to the
# blender API defined in commands.py.  The actual commands
# are transmitted to blender using the CommandListener.
#
from .. import commands
from math import radians,pi
from ..CommandSource import CommandSource
import imp
imp.reload(commands)

import queue

from .helpers import soft_import
rospy = soft_import('rospy')
std_msgs = soft_import('std_msgs.msg')
msg = soft_import('blender_api_msgs.msg')
paumsg = soft_import('pau2motors.msg')

# This is called when the CommandListener is started.
def build():
	if not rospy:
		raise ImportError('ROS not found')
		return None
	elif not msg:
		raise ImportError('Package blender_api_msg not found')
		return None
	elif not paumsg:
		raise ImportError('Package pau2motors not found')
		return None

	return RosNode()

# RosNode implements the virtual class CommandSource that blender
# expects us to use as the API to the rig.
class RosNode(CommandSource):
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
		for topic in live_topics:
			topic.publish()

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
	def __call__(self, cmd_func):
		self.cmd_func = cmd_func
		return self
	def register(self): raise NotImplementedError
	def drop(self): raise NotImplementedError

class publish_once(CommandDecorator):
	def register(self):
		self.pub = rospy.Publisher(self.topic, self.dataType, queue_size=1, latch=True)
		self.pub.publish(self.cmd_func())
	def drop(self):
		self.pub.unregister()

class publish_live(CommandDecorator):
	def register(self):
		self.pub = rospy.Publisher(self.topic, self.dataType, queue_size=1)
	def publish(self):
		self.pub.publish(self.cmd_func())
	def drop(self):
		self.pub.unregister()

class subscribe(CommandDecorator):
	def register(self):
		self.sub = rospy.Subscriber(self.topic, self.dataType, self._handle)
	def drop(self):
		self.sub.unregister()
	def _handle(self, msg):
		self.callback(IncomingCmd(self.cmd_func, msg))


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
		return msg.GetAPIVersion(commands.getAPIVersion())


	@publish_once("~available_emotion_states", msg.AvailableEmotionStates)
	def availableEmotionStates():
		return msg.AvailableEmotionStates(commands.availableEmotionStates())


	@publish_live("~get_emotion_states", msg.EmotionStates)
	def getEmotionStates():
		return msg.EmotionStates([
			msg.EmotionState(name, vals['magnitude'], rospy.Duration(vals['duration']))
			for name, vals in commands.getEmotionStates().items()
		])

	# Message is a single emotion state
	@subscribe("~set_emotion_state", msg.EmotionState)
	def setEmotionState(msg):
		emotion = str({
			msg.name: {
				'magnitude': msg.magnitude,
				'duration': msg.duration.to_sec()
			} 
		})
		commands.setEmotionState(emotion)


	@publish_once("~available_gestures", msg.AvailableGestures)
	def availableGestures():
		return msg.AvailableGestures(commands.availableGestures())


	@publish_live("~get_gestures", msg.Gestures)
	def getGestures():
		return msg.Gestures([
			msg.Gesture(
				name, vals['magnitude'], rospy.Duration(vals['duration']), vals['speed']
			) for name, vals in commands.getGestures().items()
		])


	@subscribe("~set_gesture", msg.SetGesture)
	def setGesture(msg):
		try:
			commands.setGesture(msg.name, msg.repeat, msg.speed, msg.magnitude)
		except TypeError:
			print('Error: unknown gesture:', msg.name);


	# Location that Eva will look at.
	@subscribe("~set_primary_target", msg.Target)
	def setPrimaryTarget(msg):
		# Eva uses y==forward x==right. Distances in centimeters from
		# somewhere in the middle of the head.
		# Standard robotics (e.g. player/gazebo) uses x==forward, y==left.
		# and distances in meters.  So convert.
		flist = [-100.0*msg.y, 100.0*msg.x, 100.0*msg.z]
		commands.setPrimaryTarget(flist)


	# Publishes Pau messages
	@publish_live("~get_pau", paumsg.pau)
	def getPau():
		msg = paumsg.pau()

		head = commands.getHeadData()
		msg.m_headRotation.x = head['x']
		msg.m_headRotation.y = head['y']
		msg.m_headRotation.z = head['z']
		msg.m_headRotation.w = head['w']

		eyes = commands.getEyesData()
		msg.m_eyeGazeLeftPitch = eyes['l']['p']
		msg.m_eyeGazeLeftYaw = eyes['l']['y']
		msg.m_eyeGazeRightPitch = eyes['r']['p']
		msg.m_eyeGazeRightYaw = eyes['r']['y']
		shapekeys = commands.getFaceData()

		msg.m_coeffs = shapekeys.values()
		return msg



