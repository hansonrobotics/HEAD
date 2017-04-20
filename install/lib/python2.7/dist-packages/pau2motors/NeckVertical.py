#! /usr/bin/env python3
#
# Convert vertical animator angles to mechanical angles.
#
# Copyright (c) 2015 Hanson Robotics
# Linas Vepstas February 2015

from math import sin, cos, tan, atan2, pi
import logging

logger = logging.getLogger('hr.pau2motors.neckvertical')

# Return the 3x3 identity matrix
def ident():
	result = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
	return result

# Multiply two 3x3 matrices
def matrix_mult(A, B):
	result = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
	for i in range(3):
		for j in range(3):
			for k in range(3):
				result[i][j] += A[i][k] * B[k][j]
	return result

# Rotate in the positive direction around the z-axis
def rotate_z(angle):
	c = cos(angle)
	s = sin(angle)
	return [[c, -s, 0], [s, c, 0], [0, 0, 1]]

# Rotate in the positive direction around the y-axis
def rotate_y(angle):
	c = cos(angle)
	s = sin(angle)
	return [[c, 0, s], [0, 1, 0], [-s, 0, c]]

# Print a matrix
def matrix_prt(A) :
	for row in A:
		logger.info(row)


# Given the lower-neck animator angles phi, theta, psi and the cant
# angle kappa, return the mechanical assemblage angles, as described in
# the documentation in the neck_kinematics repo.
#
# What this code does is to rock the lower u-joint backwards by an
# angle kappa, so that the upper u-joint is positioned directly above
# the lower u-joint, when the animator specifies a value of (0,0,0)
# for phi-theta-psi. Thus, whe the animator has the neck exactly
# vertical, the actual mechanical assembly has to be rocked backward
# by an angle kappa.
def neck_cant(phi_A, theta_A, psi_A, kappa):

	# The formulas and notation given in the docs
	R_phi = rotate_z(phi_A)
	R_theta = rotate_y(theta_A)
	R_psi = rotate_z(psi_A)
	R_k = rotate_y(kappa)

	S = R_k
	S = matrix_mult(R_psi, S)
	S = matrix_mult(R_theta, S)
	S = matrix_mult(R_phi, S)

	# psi plus eta
	# Degenerate case has zero in denominator.
	# Note the minus signs. Although they cancel when divided,
	# they do move the tangent to a different quadrant.
	ppe = 0
	try:
		if 0 <= theta_A + kappa:
			ppe = - atan2(-S[2][1], -S[2][0])
		else:
			ppe = - atan2(S[2][1], S[2][0])
	except ZeroDivisionError:
		ppe = 0

	R_ppe = rotate_z(-ppe)
	T = matrix_mult(S, R_ppe)

	theta = - atan2(T[2][0], T[2][2])
	phi = - atan2(T[0][1], T[1][1])
	tanphi = - T[0][1] / T[1][1]

	psi = - atan2 (tanphi, cos(theta))
	eta = ppe - psi

	return [phi, theta, eta]

# Some fairly basic sanity checking
def unit_test():
	kappa = atan2(8.93, 112.16)

	# Kappa should equal theta
	angs = neck_cant(0, 0, 0, kappa)
	if 1.0e-10 < abs(angs[1] - kappa):
		raise ArithmeticError("Bad neck math (kappa)")

	# Theta should be offset by kappa
	for x in [0.1*i for i in range(-8, 8)]:
		angs = neck_cant(0, x, 0, kappa)
		if 1.0e-10 < abs(angs[1] - (kappa + x)):
			raise ArithmeticError("Bad neck math (theta)")

	# Rotating around phi, psi makes no difference when theta=0
	for x in [0.1*i for i in range(15)]:
		angs = neck_cant(x, 0, 0, kappa)
		bangs = neck_cant(0, 0, x, kappa)
		for i in range(3):
			if 1.0e-10 < abs(angs[i] - bangs[i]):
				raise ArithmeticError("Bad neck math")

	# Levelling the rig so thtat the offset JU is horizontal should
	# cause a pair of equal offseting positive rotations.  To understand
	# this, recall that the animator phi is around the OU axis.
	for x in [0.1*i for i in range(-8, 8)]:
		angs = neck_cant(x, -kappa, 0, kappa)
		if 1.0e-10 < abs(angs[0] - x):
			raise ArithmeticError("Bad neck math (horiz-phi)")
		if 1.0e-10 < abs(angs[1]):
			raise ArithmeticError("Bad neck math (horiz-theta)")
		if 1.0e-10 < abs(angs[2] - x):
			raise ArithmeticError("Bad neck math (horiz-psi)")

	logger.info("Unit test passed")

def main():
	unit_test()
	kappa = atan2(8.93, 112.16)
	logger.info("kappa = ", kappa)
	logger.info("---")

	phi = 1.11
	psi = 1.1
	for i in range(12) :
		theta = 0.1 - i * 0.01
		angs = neck_cant(phi, theta, psi, kappa)
		logger.info("duude angles=", angs)
		logger.info("---")

# main()
