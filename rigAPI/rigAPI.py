#
# Define the blender rig API
# This is an abstract base class

class RigAPI:

    def getAPIVersion(self):
        return None

    def isAlive(self):
        return None

	# Somatic states  --------------------------------
	# awake, asleep, drunk, dazed and confused ...
	def availableSomaStates(self):
		return None

	def getSomaStates(self):
		return None

	def setSomaState(self, emotion):
		return None

	# Emotion expressions ----------------------------
	# smiling, frowning, bored ...
	def availableEmotionStates(self):
		return None

    def getEmotionStates(self):
        return None

    def setEmotionState(self, emotion):
        return None

	# Gestures --------------------------------------
	# blinking, nodding, shaking...
	def availableGestures(self):
		return None

    def getGestures(self):
        return None

    def setGesture(self, name, repeat=1, speed=1, magnitude=0.5):
        return None

    def stopGesture(self, gestureID, smoothing):
        return None

    def getGestureParams(self):
        return None

    # Visemes --------------------------------------
    def availableVisemes(self):
        return None

    def queueVisemes(self, vis, start=0, duration=0.5, \
        rampin=0.1, rampout=0.8, magnitude=1):
        return None

    # Eye look-at targets ==========================
    # The coordinate system used is head-relative, in 'engineering'
    # coordinates: 'x' is forward, 'y' to the left, and 'z' up.
    # Distances are measured in meters.  Origin of the coordinate
    # system is somewhere (where?) in the middle of the head.
    def setFaceTarget(self, location):
        return None

    def setGazeTarget(self, location):
        return None

    # ========== info dump for ROS
    # Get Head rotation quaternion in XYZ format data structure.
    # Pitch: X (positive down, negative up)?
    # Yaw: Z (negative right to positive left)
    def getHeadData(self):
        return None

    # Get Eye rotation angles:
    # Pitch: down(negative) to up(positive)
    # Yaw: left (negative) to right(positive)
    def getEyesData(self):
        return None

    def getFaceData(self):
        return None
