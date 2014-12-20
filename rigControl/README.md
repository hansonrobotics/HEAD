

# File contents

* commands.py: Defines the formal, external public API.  No other
  files define the API, asside from CommandListener, which handles
  messages I/O

* blenderCommandListener.py: defines the API for command events

* actuators.py handles autonomous functions: breathing, blinking
  eye saccades.  Does NOT use bpy.

* animationManager.py
  uses bpy, accesses bones....

* blenderPlayback.py
  uses bpy

* blendedNum.py: Implements a numeric object that allow blending and
  smoothing of values
