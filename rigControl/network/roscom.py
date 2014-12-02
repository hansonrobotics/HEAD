import bpy
from .. import commands

import imp
imp.reload(commands)

import threading, queue

from .helpers import soft_import, underscorize
rospy = soft_import('rospy')
srv = soft_import('blender_api_msgs.srv')
msg = soft_import('blender_api_msgs.msg')

def build():
	if not rospy:
		print('ROS not found')
		return None
	elif not (srv and msg):
		print('Package blender_api_msg not found')
		return None
	else:
		return RosNode()

class RosNode:
	''' all of class state is stored in self.command_queue and self.services '''

	def __init__(self):
		self.command_queue = queue.Queue()
		rospy.init_node('blender_api')

		# Find common public methods between the 'commands' module and the
		# ServiceHandlers class. Then register as ROS services.
		command_names = {name for name in dir(commands) if name[0] != '_'}
		command_names = command_names.intersection(dir(ServiceHandlers))
		self.services = self.register_services(command_names)

	def register_services(self, command_names):
		services = []
		for name in command_names:
			# Generate service parameters
			# E.g. if name == "getApiVersion" then:
			# topic == "~get_api_version"
			topic = '~' + underscorize(name)
			# srvType == srv.GetApiVersion
			srvType = getattr(srv, name[0].upper() + name[1:])
			# handler == ServiceHandler.getApiVersion
			handler = getattr(ServiceHandlers, name)
			service = rospy.Service(topic, srvType, self.queued(handler))
			services.append(service)
		return services

	def queued(self, func):
		''' wrap threadsafety around a service handler '''
		def handler(arg):
			''' delegate handler execution to the blender coroutine and wait for results '''
			command = ThreadSafeCommand(func, arg)
			self.command_queue.put(command)
			return command.release()
		return handler

	def poll(self):
		''' blocked service call getter '''
		try:
			return self.command_queue.get_nowait()
		except queue.Empty:
			return None

	def drop(self):
		''' disable communication '''
		# We don't shutdown the actual ROS node, because restarting a ROS node after
		# shutdown is not supported in rospy.
		for service in self.services:
			service.shutdown()
		return True

class ServiceHandlers:
	'''
	The names of these methods should match the ones in 'commands' module.
	These methods shouldn't be called directly but through
	ThreadSafeCommand.execute() in the Blender coroutine.

	The returned values are implicitly converted to response objects like
	srv.GetApiVersionResponse, srv.GetEmotionStatesResponse, etc.
	See http://wiki.ros.org/rospy/Overview/Services#Providing_services for valid
	return values.
	'''

	@staticmethod
	def getAPIVersion(req):
		return commands.getAPIVersion()

	@staticmethod
	def availableEmotionStates(req):
		return [commands.availableEmotionStates()]

	@staticmethod
	def getEmotionStates(req):
		return [[
			msg.EmotionState(name, value) for name, value in
			commands.getEmotionStates(bpy.evaAnimationManager).items()
		]]

	@staticmethod
	def setEmotionStates(req):
		emotions = {emotion.name: emotion.value for emotion in req.data}
		return commands.setEmotionStates(emotions, bpy.evaAnimationManager) or 0

	@staticmethod
	def availableEmotionGestures(req):
		return [commands.availableEmotionGestures()]

	@staticmethod
	def getEmotionGestures(req):
		return [[
			msg.EmotionGesture(name, rospy.Duration(time)) for name, time in
			commands.getEmotionGestures(bpy.evaAnimationManager)
		]]

class ThreadSafeCommand:

	def __init__(self, func, arg):
		self.func, self.arg = func, arg
		self.condition = threading.Condition()
		self.is_executed = False

	def execute(self):
		''' should be called from the Blender coroutine '''
		with self.condition:
			self.response = self.func(self.arg)
			self.is_executed = True
			self.condition.notify_all()

	def release(self):
		''' should be called from a ROS service handler coroutine '''
		with self.condition:
			self.condition.wait_for(lambda: self.is_executed)
			return self.response
