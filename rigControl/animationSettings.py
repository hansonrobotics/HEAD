# defines the max and min rate for eye darts (in hertz)
# to be mapped to the normalized value (-1 to +1)
MIN_EYE_DART_RATE = 0.01
MAX_EYE_DART_RATE = 3

# defines the max and min distance for eye darts (as gaussian distribution sigma)
# to be mapped to the normalized value (0 to +1)
MIN_EYE_WANDER = 0.0
MAX_EYE_WANDER = 0.1

# defines the max and min rate for eye blink (as hz)
# to be mapped to the normalized value (0 to +2)
MIN_BLINK_RATE = 0.1
MAX_BLINK_RATE = 0.8  	# technically 24 times a min is 0.4hz, using 0.8 hz for debugging

# define the speed of the breathing animation (as normalized to the 1x speed)
MIN_BREATH_RATE = 0.5
MAX_BREATH_RATE = 3.0

# define the strength of the breathing animation
MIN_BREATH_INTENSITY = 0
MAX_BREATH_INTENSITY = 2.0
