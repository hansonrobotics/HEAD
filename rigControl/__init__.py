#

import importlib
#
# If there is no ROS, then don't initalize the command listener
haveROS = True
try:
	importlib.import_module('rospy')
except ImportError:
	haveROS = False

if haveROS:
	from . import blenderCommandListener

from . import blenderUI
from . import blenderPlayback
from . import animationManager

import imp

# sets up the Command listener
if haveROS:
	imp.reload(blenderCommandListener)
	blenderCommandListener.refresh()

# sets up the Blender operators
imp.reload(blenderPlayback)
blenderPlayback.refresh()

# sets up the Blender interface
imp.reload(blenderUI)
blenderUI.refresh()

# init animation Manager singleton
imp.reload(animationManager)
animationManager.init()
