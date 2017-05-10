#
# Define the blender rig API
# This is an abstract base class

class RigAPI:

    def getAPIVersion(self):
        return None

    def isAlive(self):
        return None
    # Codes to enable faceshift to blendshape mapping
    def getAnimationMode(self):
        return None

    def setAnimationMode(self,animation_mode=0):
        return None

    def setShapekeys(self, shapekeys):
        return None

    def setAnimationMode(self,animation_mode=0):
        return None

    def setShapekeys(self, shapekeys):
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
    # Speed is added for more control. Uses moving average over period of time
    def setFaceTarget(self, location, speed=1.0):
        return None

    def setGazeTarget(self, location, speed=1.0):
        return None

    # Head roll ----------------------------------
    # Sets head roll by the angle given.
    def setHeadRotation(self, location):
        return None

    # ========== info dump for ROS
    # Get Head and Neck rotation quaternion in XYZ format data structure.
    # Its assumed that the robot neck assembly has two joints; the upper
    # joint is the "head" joint, the lower one is the "neck" joint.
    #
    # Pitch: Z (positive down, negative up)
    # Yaw: X (negative left to positive right)
    #
    # XXX TODO It does not really make sense to specify two quaternions,
    # without also specifying the physical distance between the two
    # joints.  Right now, this distanc is hard-coded in various random
    # places, but this should be fixed in some way.  (For example, maybe
    # the blender rig should use physical dimensions queried from the
    # actual robot being modelled??)
    def getHeadData(self):
        return None

    def getNeckData(self):
        return None

    # Get Eye rotation angles:
    # Pitch: down(negative) to up(positive)
    # Yaw: left (negative) to right(positive)
    def getEyesData(self):
        return None

    def getFaceData(self):
        return None

    # Set the lower neck rotation
    # Accepts pitch and roll as parameters for the neck
    def setNeckRotation(self, pitch, roll):
        return None

    def setParam(self, key, value):
        return None

    def getParam(self, param):
        return None

    def getAnimationLength(self, animation):
        return None
