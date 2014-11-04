from . import blenderUI
from . import blenderCommandListener
from . import blenderPlayback
# from . import network

import imp

# sets up the Command listener
imp.reload(blenderCommandListener)
blenderCommandListener.refresh()

# sets up the Blender interface
imp.reload(blenderUI)
blenderUI.refresh()

# sets up the animation playback
imp.reload(blenderPlayback)
blenderPlayback.refresh()