Blender_API v1.0 (draft)
================
The Blender_API defines the Python interface to controlling the blender
rig. The corresponding ROS node interface can be found in cookbook.md

The API is implemented in the `rigControl/commands.py` file.

##Initializtion and Termination
General status reporting.

---

`getAPIVersion()`

Returns a positive integer that represents the version of the currently
running CommandListener. Used to make sure the server and the client
are running on the same version of the API. Current version is 1.
Takes no argument.

---
`init()`

Initializes the 3D environment. Must be called before any other command
except for getAPIVersion().  Takes no argument.  Returns 0 if success,
else returns an error code.

---
`getEnvironment()`

Returns the parameters of the currently initialized environment. Things
returned could include nominal playback framerate, commandListener pull
rate, statistics on performance, system hardware, etc.

Can be also used to as a way to compute round trip time between sending
a command and getting a reply. Useful for time critical communication
such as speech, where a minuscule time delay can look unnatural.

---
`isAlive()`

Returns 1 if 3D environment is alive and ready to accept command, else
returns 0.  Should be called after `init()` to make sure
that blender is properly operating, and that the 3D environment is ready
to accept further commands.

---

`terminate()`

Terminates the 3D environment and severs the connection.  To use the
API again, the connection needs to be re-establshed with `init()`.
Returns 0 if success, else returns an error code.  Takes no argument.

---

##Emotion & Gesture

Emotional facial expressions are long-duration movements and expressions
shown on the head and face, such as happiness, sadness.  Directly
controllable gestures are short-duration movements, such as nods, shakes
and blinks. Autonomic gestures, such as breathing motion, cannot be
stopped.

The functions here report a menu of available emotional states and
gestures, starts ans stops them, and reports the current emotional and
gestural state.

---

`availableEmotionStates()`

Returns a list of the available emotion states, in string format.
The list of available animations depends on the specific blender
rig; that is, it depends on what has been impleemnted.  Typical
emotional states include `sad`, `happy`, `confused`, etc.

Takes no argument.

---
`getEmotionStates()`

Returns the current state of the character’s emotional state, as a list
of objects.  The list enumerates all of the emotions being curently
expressed, how long each has been expressed, and the magntidue
(strength) of the expression.  The progression of emotional states
depends on the rig.  Typically, the rig can express multiple
different emotion types at once (e.g. surprise and happiness),
with varying degrees of strength, the the strngth decaying over time
to a neutral state.

Takes no argument.

---

`setEmotionState(emotion)`

Adds the indicated emotion to the set of emotions being currently
expressed.  The emotion is a triple, consisting of the emotion name,
the strength of the expression, and the duration for which it should
be expressed. The name should be one of the values returned by
`availableemotionStates()`.

The new emotion is blended into the current emotion state, and is
decayed to neutral over the given period of time.

To alter a currently playing emotion, simply specify a new state
for it.

Returns 0 if success, else returns the error code.

---

`availableGestures()`

Returns a list of the available gestures in string format.
These typically include gestures such as yawn, nod, and blink.

---

`getGestures()`

Returns the current state of the character’s gesture state.  This
includes the name of the gesture, the magnitude of its expression,
and the time remaining for each gesture.  The list includes both
the specifiable gestures (nodding, blinking) as well as autonomic
ones (breathing).

---

`setGesture(“gesture” = string, “repeat”= float, “speed” = float,
    “magnitude” = float)`

Triggers an immediate playback of a gesture. Parameters include:

* gesture: the string name of the gesture.
* repeat: the number of times the gesture is to be repeated (1 being default)
* speed: the playback speed of the gesture (1 being nominal, 2 is twice as
   fast, etc)
* magnitude: the intensity of the gesture (1 being nominal, 0.5 being
  half as intense, etc.)
* Priority: proposed, not implemented.

Returns 0 if success, else returns the error code.

---
`stopGesture(gestureID = id, smoothing = float)`

Stops an existing gesture. Used to interrupt one gesture (for another).
Returns 0 when the command is successfully registered
(not when gesture is stopped). Returns error code otherwise.

XXX TODO Not implemented.

---
`getGestureParams()`

---

##Target Tracking and Speech
Commands for tracking movements of the eyes, and lip-sync motions.

---

`setFaceTarget(“location” = vec3, “scale” = vec3, “dart_rate” = float, “rotation” = vec3)`

Specify a visual target to look at and face.

The coordinate system used is torso-relative, in 'engineering'
coordinates: 'x' is forward, 'y' to the left, and 'z' up.
Distances are measured in meters.  Origin of the coordinate
system is somewhere (where?) in the middle of the head.

* `location` controls where the character will look and turn.

* `scale` is the size of the target. Affects how much the eyes drift
  around when examining a target.

* `dart_rate` is the rate (how often) the eyes will break contact
  with the gaze target.

* `rotation` is the rotation of the target, can affect head tilt when
  the character is in a playful mood. (Not implemented at this time).

Returns 0 if success, else returns the error code.

---
`setGazeTarget(“location” = vec3, “scale” = vec3, “dart_rate” = float)`

Specify a visual target to look at, using the eyes only, without turning
the head.

* `location` controls where the character will look and turn.

* `scale` is the size of the target. Affects how much the eyes drift
  around when examining a target.

* `dart_rate` is the rate (how often) the eyes will break contact
  with the gaze target.


Returns 0 if success, else returns the error code.

---

