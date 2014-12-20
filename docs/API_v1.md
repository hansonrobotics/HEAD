Blender_API v1.0 (draft)
================
The Blender_API is implemented in the `rigControl/commands.py` file.

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

Returns 0 if 3D environment is alive and ready to accept command, else
returns the error code.	Should be called after `init()` to make sure
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
shown on the head and face, such as happiness, sadness.  Gestures are
short-duration movements, such as nods, shakes and blinks.

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

`setEmotionStates(emotion)`

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

`availableEmotionGestures()`

Returns a list of the available emotionGestures in string format	Ensures the AI modules is aware of what emotionGestures are supported (i.e. yawn, nod, blink)

---

`getEmotionGestures()`

Returns the current state of the character’s emotionGestures that’s playing (including time remaining for each gesture) as a list-of-strings format	emotionGestures are manually triggered animations such as yawn, nod. 

---

`setEmotionsGestures(     “gesture” = string     “repeat” = float,      “speed" = float,
    “magnitude" = float,     “priority" = float)"`
    
Returns 0 if success, else returns the error code	"triggers an immediate playback of a gesture. Parameters include:

gesture: the string representing the name of the gesture
repeat: the number of times the gesture is to be repeated (1 being default)
speed: the playback speed of the gesture (1 being nominal, 2 is 2x as fast, etc)
magnitude: the intensity of the gesture (1 being nominal, 0.5 being half as intense, etc)

---
`stopEmotionGestures(  gestureID = id,  smoothing = float
)
`
Returns 0 when the command is successfully registered (not when gesture is stopped). Returns error code otherwise.	Stops an existing gesture. Used to interrupt one gesture (for another).

---	
		
Target Tracking and Speech
---


`setPrimaryTarget(
    “id” = int,     “location” = vec3,     “rotation” = vec3,     “scale” = vec3,
    “tracking” = 0.7 )`
    
Returns 0 if success, else returns the error code	"id is the identifier for each unique target. Useful for when eva is interacting with multiple people.

location controls where the character will look and turn
rotation is the rotation of the target, can affe head tilt when the character is in a playful mood scale is the size of the target. affects how much the eyes drift around when locked onto a target
tracking controls the percentage of times when the character is looking at this target"

---
`setSecondaryTarget(
    “id” = int,     “location” = vec3,     “rotation” = vec3,     “scale” = vec3,     “tracking” = 0.3 )`
    
Returns 0 if success, else returns the error code	"id is the identifier for each unique target. Useful for when eva is interacting with multiple people.

location controls where the character will look and turn
rotation is the rotation of the target, can affe head tilt when the character is in a playful mood scale is the size of the target. affects how much the eyes drift around when locked onto a target
tracking controls the percentage of times when the character is looking at this target"

---
`engageTarget( target = primary|secondary
time = float )`

Returns 0 if success, else returns the error code	Force character to look at a specific target for time.

---

`saySpeech()` TBD	TBD
		
		
