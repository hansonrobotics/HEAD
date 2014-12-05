# Implements helper functions used by the rest of the module

import math

def mix(a,b,factor):
	'''mix two number together using a factor'''
	if type(a) is list or type(a) is tuple:
		if len(a)==len(b)==2:
			return [a[0]*factor + b[0]*(1.0-factor), a[1]*factor + b[1]*(1.0-factor)]
		elif len(a)==len(b)==3:
			return [a[0]*factor + b[0]*(1.0-factor), a[1]*factor + b[1]*(1.0-factor), a[2]*factor + b[2]*(1.0-factor)]
		elif len(a)==len(b)==4:
			return [a[0]*factor + b[0]*(1.0-factor), a[1]*factor + b[1]*(1.0-factor), a[2]*factor + b[2]*(1.0-factor), a[3]*factor + b[3]*(1.0-factor)]
		else:
			raise Exception(ArithmeticError)
	else:
		return (a*factor + b*(1.0-factor))


def checkValue(var, minV, maxV):
	if not minV <= var <= maxV:
		print('Warning:', var, 'is outside of the expected range ', ':', minV, '-' , maxV)


def smoothstep(x):
	'''returns a smoothed float given an input between 0 and 1'''
	return x * x * (3-2*(x))


def computeDistance(a, b):
	if len(a)==len(b)==2:
		return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)
	elif len(a)==len(b)==3:
		return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)
	elif len(a)==len(b)==4:
		return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2 + (a[3]-b[3])**2)
	else:
		raise Exception(ArithmeticError)


def mapValue(value, leftMin, leftMax, rightMin, rightMax):
	# map value from one range to another range

	# Figure out how 'wide' each range is
	leftSpan = leftMax - leftMin
	rightSpan = rightMax - rightMin

	# Convert the left range into a 0-1 range (float)
	valueScaled = float(value - leftMin) / float(leftSpan)

	# Convert the 0-1 range into a value in the right range.
	return rightMin + (valueScaled * rightSpan)
