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
