# Implementation Internals
The main user interface is implemented in the `commands.py` file, and
is documented in the [../docs/API_v1.md](../docs/API_v1.md) file.  See
also the ROS HOWTO tutorial in [../docs/cookbook.md](../docs/cookbook.md).
What follows below is documentation for the internal implementation.
It is not meant to be accessed directly by users.

## File contents

* commands.py: Defines the formal, external public API.  No other
  files define the API, asside from CommandListener, which handles
  the polling for command messages, and the publishing of status data.

* CommandSource.py: defines the API for command events. New sources of
  commands should inherit from this class. The ROS node uses this, to
  convert the ROS messages into API calls.

* CommandListener.py: command handler, accepts events from one or
  more command sources, and queues them with blender.

* blenderUI.py: shim to wire up the emotion/gesture selection panel
  in the blender window, to commands.py.  So, when a button is clicked
  in the blender panel, the code here causes the corresponding
  commands.py API call to be called.

* animationManager.py: provides KeepAlive method, and meths to start
  an emotion or a gesture.  Uses bpy, accesses bones....

* actuators.py: algorithmic scripted behaviors for a handful of
  autonomous functions: breathing, blinking, eye saccades, head drift.
  Does NOT use bpy directly, uses AnimationManager.

* blenderPlayback.py ... per-frame service
  uses bpy

* blendedNum.py: Implements a numeric object that allow blending and
  smoothing of values

##Actions
Rig modifications are implemented as actions. These are not meant to be
invoked manually; use teh commands.py interface for that. The actions
constitute the lowest-level interace between blender and the python
code in this directory.

There are several kinds of actions:

* Cycle actions (prefix: CYC) - breathing, normal and sleeping
  actions. Used to for keep-alive movement.
* Emotion actions (prefix: EMO) - Emotion actions consisting of about
  100 frames each, which represent verious emotional expressions. Emotion
  actions are controlled by drivers associated with bones.
* Gesture actions (prefix: GST) - Gestures that can be called from API
  and applied over the CYC and EMO actions.
* Other Gestures - some other actions defined but not yet used with
  API yet. Like LipSync actions.

##Rig Architecture

The Eva rig contains two separate armatures; "deform" and "control". The
"deform" armature is to be used by the animator when creating poses and
actions. The "control" armature contains the automated controls which
are not manually animated.

###Deform armature

Most of the controls on the deform armature drive shape keys on the Eva
mesh. These controls are supposed to directly relate to the motors in
the actual robot and are found on layers 1 and 2 of the armature. In
addition, there are special controls for blinks and lip seal. The blink
controllers are located on the eyes and are red in colour. The lip seal
control sits to the right (screen left) of the head. The reason for
these extra controls is to ensure that blinks and lip seal always result
in 100% lip or eye closure, regardless of the starting point.

There are also controls for eye offset ('eye_offset', 'eye_R' and
'eye_L') and head offset ('head'). These controls are parented to the
automated head and eye targets. They are to be used by the animator when
adjustment to the default position of the eyes and head are required.

Below the head are a series of biometric controls that don't directly
control the rig, but instead control certain parameters of Eva's
behaviour. The API reads these values to determine certain parameters.
They are: eye dart rate, eye wander, blink rate, blink duration, breath
rate and breath intensity. These are mainly to be used for the different
emotional states, but can also be used in gestures and other actions.

The deformation bones are DEF_neck, DEF_head, DEF_jaw, eye.L and eye.R.
These bones provide rotational values to the robot and are not to be
manipulated by the animator.

###Control armature

This contains the head_target, eye_target and the emotion controllers.
The emotion controllers drive action constraints on the controllers in
the deform rig. Each has a corresponding action which represents an
intensity level of 0-100. The intensity of the action constraints is
directly driven by the X location of the controller. The X location is
in turn driven by a special property called "intensity", which is
manipulated by the API.

(The reason for this roundabout route is a limitation in Blender's
action constraints; they only work on transforms, not on special
properties.) In addition there are some dummy controllers that serve as
labels for the biometric controls. They are locked off, and are only
there for ease of use by the animator. They can be ignored by anyone
else.
