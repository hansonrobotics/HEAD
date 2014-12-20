#

from . import blenderCommandListener
from . import blenderUI
from . import blenderPlayback
from . import animationManager

import imp

# sets up the Command listener
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

# Temporary hack for loading the ROS node
# If ROS is found, we try to load the ROS node, else not.
import importlib
try:
	importlib.import_module('rospy')
	from .rosrig import init
	node = rosrig.init()
	blenderCommandListener.register_cmd_source(node)
except:
	print('No ROS found')
