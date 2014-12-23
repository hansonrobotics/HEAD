# File contents

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

#Actions
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
