__author__ = 'vytas'
from .blendedNum import BlendedNum
from math import atan, tan, sqrt
from time import time


class BlendedAngle(BlendedNum):
	''' Smoothens direction for vector3 '''

	# Converts location to direction and distance
	@staticmethod
	def location2angles(value):
		a = [0,0,0]
		# Yaw
		a[0] = atan(value[0]/value[1])
		# Pitch
		a[1] = atan(value[2]/value[1])
		# length
		a[2] = sqrt(value[0]**2+value[1]**2+value[2]**2)
		return a
	# Converts direction and distance back to coordinates
	@staticmethod
	def angles2location(angles):
		v = [0,0,0]
		v[1] = angles[2]/sqrt(1+tan(angles[0])**2+tan(angles[1])**2)
		v[0] = v[1] * tan(angles[0])
		v[2] = v[1] * tan(angles[1])
		return v

	# Two additional parameters added:
	# maximum ang_speed in rad/s
	# offset = [0,0,0] to calculate proper angle of head rotations
	#  and coordinates offset
	def __init__(self, value, steps = 20, smoothing = 20, ang_speed=1, offset=[0,0,0]):
		# only deal with loc vectors
		self._time = time()
		self._ang_speed = ang_speed
		self._offset = offset
		# Add offset to calculate
		v = [x-y for x,y in zip(value,offset)]
		a = self.location2angles(v)
		self._min_steps = steps
		self._angle_max = 0
		super(BlendedAngle,self).__init__(a, steps, smoothing)

	@property
	def current(self):
		a = super(BlendedAngle,self).current
		return [x+y for x,y in zip(self.angles2location(a), self._offset)]

	@property
	def target(self):
		a = super(BlendedAngle,self).target
		return [x+y for x,y in zip(self.angles2location(a), self._offset)]

	@target.setter
	def target(self, value):
		v = [x-y for x,y in zip(value,self._offset)]
		a = self.location2angles(v)
		# consider only angles and ignore distance
		self._angle_max = max(abs(x-y) for x,y in zip(a[:2],self._current[:2]))
		self._target = a
		self._old = self._current.copy()

	@property
	def ang_speed(self):
		return self._ang_speed

	@ang_speed.setter
	def ang_speed(self, value):
		self._ang_speed = value

	def blend(self):
		# Set the steps based on the max angle in current turn
		dt = time()-self._time
		self._steps = max(self._min_steps, self._angle_max/(float(self._ang_speed) * dt))
		self._time = time()
		BlendedNum.blend(self)
