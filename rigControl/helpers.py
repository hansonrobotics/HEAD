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
