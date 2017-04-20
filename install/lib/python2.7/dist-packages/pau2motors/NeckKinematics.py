#
# Double U-joint neck inverse kinematics.
# Copyright (c) 2015 Hanson Robotics
#
# Linas Vepstas January 2015
#
from math import sin, cos, sqrt, atan2, asin
import sys

# Distance
def dist(x, y, z):
	return sqrt(x*x + y*y + z*z)

# Quadratic trig equation solver. Returns the two roots of the equation
# alpha * sin(theta) + beta * cos(theta) + gama = 0
# The above can be thought of as a quadratic equation in sin(theta)
# and so can be solved as an ordinary quadratic. Finally, return
# arcsin(theta) as the angle that solves the eqn.
def quad_trig(alpha, beta, gamma, sign):
	asq = alpha * alpha
	bsq = beta * beta
	csq = gamma * gamma
	det = asq * csq + (bsq + asq) * (bsq - csq)
	if det < 0.0:
		raise OverflowError("Linkage jamming!")
	det = sqrt(det)
	sinth = (- alpha * gamma + sign * det) / (asq + bsq)
	return asin(sinth)

# Newtonian interpolation. Finds the zero of function func.
# Assumes that there is one unique zero.  Assumes that func(pi/2)
# is of opposite sign from func(-pi/2).  Throws an exception if
# there are no solutions. In the context of the neck linkage, this
# means that a neck position was requested that the mechanical linkage
# simply cannot get to; even at full extension, the neck cannot be moved
# to there.
def newton(func):
	# Newton's method
	ta = -3.1415 * 0.5
	tb = 3.1415 * 0.5
	da = func(ta)
	db = func(tb)

	if 0.0 <= da*db :
		raise OverflowError("Linkage jamming!")

	while True:
		islo = (tb - ta) / (db - da)
		tm = ta - da * islo
		dm = func(tm)
		if abs(dm) < 10.e-14:
			return tm
		if 0.0 <= dm*da:
			da = dm
			ta = tm
		else:
			db = dm
			tb = tm

# Inverse kinematics solver for the Eva/Han neck linkage mechanism.
# Given desired sphere angles theta, phi for the neck U-joint
# orientation, the inverse_kinematics(theta, phi) will compute the
# motor angles to achieve that position.  The two motor angles will
# be set in self.theta_l and self.theta_r i.e. the left and right
# motors.  Rotation follows the right-hand-rule.  See the accompanying
# documentation in PDF or LyX format to see the drawings that these
# dimensions and angles refer to.
class neck_linkage:

	# Initialization. None.  Must be duck-typed correctly.
	def __init__(self):
		pass

	# Return the distance between points G and F
	# More precisely, given a motor angle theta_r, compute the
	# distance between motor arm joint G and the fixed location of F,
	# and compare this to the desired distance L_FG
	def rmoto(self, theta_r) :
		gx = self.hx
		gy = self.hy + self.lgh * cos(theta_r)
		gz = self.hz + self.lgh * sin(theta_r)

		gx -= self.fx
		gy -= self.fy
		gz -= self.fz

		delta = self.lfg - dist(gx, gy, gz)
		return delta

	# Return the distance between points C and D
	# More precisely, given a motor angle theta_l, compute the
	# distance between motor arm joint D and the fixed location of C,
	# and compare this to the desired distance L_CD
	def lmoto(self, theta_l) :
		dx = self.ex
		dy = self.ey - self.lde * cos(theta_l)
		dz = self.ez - self.lde * sin(theta_l)

		dx -= self.cx
		dy -= self.cy
		dz -= self.cz

		return self.lcd - dist(dx, dy, dz)

	# Compute the left-hand motor angle, given the desired location for
	# point C.  That is, this assumes that self.cx, cy, cz have been set
	# up to the desired location. Returns the motor angle that provides
	# that location.
	def langle(self) :
		alpha = self.cz - self.ez
		beta = self.cy - self.ey
		gamma = self.cx - self.ex
		lce = dist(alpha, beta, gamma)
		lcesq = lce * lce
		ldesq = self.lde * self.lde
		lcdsq = self.lcd * self.lcd
		gamma = - 0.5 * (lcdsq - ldesq - lcesq) / self.lde

		return quad_trig(alpha, beta, gamma, 1)

	# Compute the right-hand motor angle, given the desired location for
	# point F.  That is, this assumes that self.fx, fy, fz have been set
	# up to the desired location. Returns the motor angle that provides
	# that location.
	def rangle(self) :
		alpha = self.fz - self.hz
		beta = self.fy - self.hy
		gamma = self.fx - self.hx
		lfh = dist(alpha, beta, gamma)
		lfhsq = lfh * lfh
		lghsq = self.lgh * self.lgh
		lfgsq = self.lfg * self.lfg
		gamma = - 0.5 * (lfgsq - lghsq - lfhsq) / self.lgh

		return quad_trig(-alpha, beta, gamma, -1)

	# Solve the inverse kinematics problem for the lower neck motors.
	# Given spherical angles theta, phi for the neck stem, compute
	# the motor angles (in radians) for the left and right motors.
	# The left and right motor angles are placed in self.theta_l and
	# self.theta_r
	def inverse_kinematics(self, theta, phi) :
		cf = cos(phi)
		sf = sin(phi)

		ct = cos(theta)
		st = sin(theta)

		# Rotate C by psi
		psi = - atan2 (ct * sf, cf)
		cp = cos(psi)
		sp = sin(psi)
		cx = cp * self.cx0 - sp * self.cy0
		cy = sp * self.cx0 + cp * self.cy0
		cz = self.cz0

		# Rotate C by theta
		cpx = ct * cx + st * cz
		cpy = cy
		cpz = -st * cx + ct * cz

		# Rotate C by phi
		self.cx = cf * cpx - sf* cpy
		self.cy = sf * cpx + cf* cpy
		self.cz = cpz

		# Rotate F by psi
		fx = cp * self.fx0 - sp * self.fy0
		fy = sp * self.fx0 + cp * self.fy0
		fz = self.fz0

		# Rotate F by theta
		fpx = ct * fx + st * fz
		fpy = fy
		fpz = -st * fx + ct * fz

		# Rotate F by phi
		self.fx = cf * fpx - sf * fpy
		self.fy = sf * fpx + cf * fpy
		self.fz = fpz

		# Left and right motor angles
		self.theta_l = self.langle()
		self.theta_r = self.rangle()

		# Double-check, using Newton's method
		the_l = newton(self.lmoto)
		the_r = newton(self.rmoto)
		if abs(self.theta_l - the_l) > 1.0e-10:
			raise ArithmeticError("Miscalculation of left motor angle!")
		if abs(self.theta_r - the_r) > 1.0e-10:
			raise ArithmeticError("Miscalculation of right motor angle!")

		self.theta_l -= self.theta_l_neutral
		self.theta_r -= self.theta_r_neutral


# The lower neck linkage
class lower_neck(neck_linkage):

	# Initialization: mostly just setting of sizes of the actual
	# lower neck geometry.
	def __init__(self):

		# Lengths of important dimensions
		# per Davide Recchia, taken from CAD drawing
		self.loa = 12.65

		# per Davide Recchia, taken from CAD drawing
		self.lab = 38.27

		# per Davide Recchia, taken from CAD drawing
		self.lbc = 47.68
		self.lbf = self.lbc

		# per Davide Recchia, taken from CAD drawing
		self.lcd = 50.0
		self.lfg = self.lcd

		# Length of the motor arms.
		# 13 from GST0025_HornNeckDynamixel drawing
		self.lde = 13.0
		self.lgh = self.lde

		# Location of the two motors
		# per Davide Recchia, taken from CAD drawing
		self.hx = 40.0
		self.hy = 34.2
		self.hz = -57.35

		self.ex = self.hx
		self.ey = -self.hy
		self.ez = self.hz

		# Neutral location of C and F.
		self.cx0 = self.lab
		self.cy0 = -self.lbc
		self.cz0 = -self.loa

		self.fx0 = self.lab
		self.fy0 = self.lbf
		self.fz0 = -self.loa

		# Current positions of C and F
		self.cx = self.cx0
		self.cy = self.cy0
		self.cz = self.cz0

		self.fx = self.fx0
		self.fy = self.fy0
		self.fz = self.fz0

		# Motor neutral position
		self.theta_r_neutral = -0.41528994045450957
		self.theta_l_neutral = -self.theta_r_neutral

		self.theta_r = 0.0
		self.theta_l = 0.0

# The upper neck linkage
class upper_neck(neck_linkage):

	# Initialization: mostly just setting of sizes of the actual
	# upper neck geometry.
	def __init__(self):

		# Lengths of important dimensions
		# per Davide Recchia, taken from CAD drawing
		self.loa = -1.0

		# per Davide Recchia, taken from CAD drawing
		self.lab = 25.0

		# per Davide Recchia, taken from CAD drawing
		self.lbc = 35.15
		self.lbf = self.lbc

		# per Davide Recchia, taken from CAD drawing
		self.lcd = 57.71
		self.lfg = self.lcd

		# Length of the motor arms.
		self.lde = 16.0
		self.lgh = self.lde

		# Location of the two motors
		self.hx = 25.0
		self.hy = 19.85
		self.hz = -56.71

		self.ex = self.hx
		self.ey = -self.hy
		self.ez = self.hz

		# Neutral location of C and F.
		self.cx0 = self.lab
		self.cy0 = -self.lbc
		self.cz0 = -self.loa

		self.fx0 = self.lab
		self.fy0 = self.lbf
		self.fz0 = -self.loa

		# Current positions of C and F
		self.cx = self.cx0
		self.cy = self.cy0
		self.cz = self.cz0

		self.fx = self.fx0
		self.fy = self.fy0
		self.fz = self.fz0

		# Motor neutral position
		self.theta_r_neutral = 0.2
		self.theta_l_neutral = -self.theta_r_neutral

		self.theta_r = 0
		self.theta_l = 0
