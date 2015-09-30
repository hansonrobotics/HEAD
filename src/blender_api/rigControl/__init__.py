#
# __init__.py
#
# Initialize the rigControl module
#

def init():

    from . import animationManager
    from . import blenderUI
    from . import blenderPlayback
    from . import CommandListener

    import imp

    # Goofy hack -- It seems that we are called twice, by ../loader.py.
    # The first time by `import rigControl` at line 11, and then again by
    # imp.reload(rigControl) at line 12.  This is intentional: it allows
    # the blender tdev to reload without quitting blender.  But ... I
    # don't really want to init ROS twice, so I will hack around this.
    # The very first time through, the NameError exception is thrown, and
    # ROS gets initilized. The second time through, we return w/o the init.
    try:
        stuff = is_init
    except NameError:
        is_init = False

    if is_init:
        return

    is_init = True

    # sets up the Command listener
    imp.reload(CommandListener)
    CommandListener.refresh()

    # sets up the Blender operators
    imp.reload(blenderPlayback)
    blenderPlayback.refresh()

    # sets up the Blender interface
    imp.reload(blenderUI)
    blenderUI.refresh()

    # init animation Manager singleton
    imp.reload(animationManager)
    animationManager.init()
