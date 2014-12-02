Blender_ROS_API Draft
===


System Commands
---

`getAPIVersion()`

Returns a positive integer that represents the version of the currently running CommandListener	Used to make sure the server and the client are running on the same version of the API. Initial iteration will be version 1. Takes no argument.

---
`init()`

Returns 0 if success, else returns the error code	Initializes the 3D environment, must be called before any other command except for getAPIVersion() Takes no argument currently.

---
 `getEnvironment()`

Returns the parameters of the currently initialized environment. Things returned could include nominal playback framerate, commandListener pull rate, statistics on performance, system hardware, etc.

Can be also used to as a way to compute round trip time between sending a command and getting a reply. Useful for time critical communication such as speech, where a minuscule time delay can look unnatural."
isAlive()	Returns 0 if 3D environment is alive and ready to accept command, else returns the error code	"Should be called after init() to make sure everything is kosher and that the 3D environment is ready to accept further commands.

---

`terminate()`

Returns 0 if success, else returns the error code	"Terminates the 3D environment and severs the connection, will have to reestablish it with init() after terminate() is called.

Takes no argument currently.

---

Emotion & Gesture
---


`availableEmotionStates()`

Returns a list of the available emotionState in string format. Because the list of available animations in the 3D environment is subjected to change, this call ensures the AI modules is aware of what emotionStates are supported (i.e. sad, happy, confused, etc)

Takes no argument.

---
`getEmotionStates()`

Returns the current state of the character’s emotionStates in an object format (or list of list, depending on the protocol being used)	"Because the emotionStates of the character is dynamic and interpolated, this might be slightly different than what’s expected even immediately after called setEmotionStates()

Takes no argument

---

`setEmotionStates(     emotionState1 = float,     emotionState2 = float )`

Returns 0 if success, else returns the error code	"Inside Blender, EmotionState values will quickly blend into the existing emotionStates, and slowly decay to neutral if no new values are set, this ensure the character doesn't looks stiff or freezes in an awkward pose.

emotionStatesN are any of the emotionStates returned by availableemotionStates()

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
		
		