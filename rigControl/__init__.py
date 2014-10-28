from . import blenderUI
from . import blenderCommandListener

import imp

# sets up the Blender interface
imp.reload(blenderUI)
blenderUI.refresh()

# sets up the Command listener
imp.reload(blenderCommandListener)
blenderCommandListener.refresh()
