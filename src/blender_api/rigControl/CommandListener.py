# This module sets up a modal operator in Blender to act
# as the command listener for external events

commandRateHz = 10
debug = True

import bpy

from . import commands
from rigAPI import CommandSource

import imp
imp.reload(commands)

class BLCommandListener(bpy.types.Operator):
    """Listens for external commands"""
    bl_label = "Command Listener"
    bl_idname = 'wm.command_listener'

    cmd_sources = None
    _timer = None
    bpy.types.Scene.commandListenerActive = bpy.props.BoolProperty( name = "commandListenerActive", default=False)
    bpy.context.scene['commandListenerActive'] = False

    def modal(self, context, event):
        if debug and event.type in {'ESC'}:
            return self.cancel(context)

        if debug and event.type == 'TIMER':
            # print('Running Command Listener', round(self._timer.time_duration,3))

            # Poll each possible command source, see if it has anything
            # for us to do.
            while True:
                have_more = False
                for src in self.cmd_sources:
                    command = src.poll()
                    if command:
                        command.execute()
                        have_more = True
                if not have_more:
                    break
            for src in self.cmd_sources:
                src.push()

        # set status
        bpy.context.scene['commandListenerActive'] = True
        return {'PASS_THROUGH'}


    def execute(self, context):
        print('Starting Command Listener')

        # Load cmd sources on first press of the button
        if self.cmd_sources == None:
            type(self).cmd_sources, names = load_cmd_sources()
            if len(self.cmd_sources) > 0:
                print("Command Source '%s' loaded" % ', '.join(names))
            else:
                print('No Command Sources found')

        success = True
        for src in self.cmd_sources:
            src.init()
            success = src.push()

        if success:
            wm = context.window_manager
            self._timer = wm.event_timer_add(1/commandRateHz, context.window)
            wm.modal_handler_add(self)
            bpy.context.scene['commandListenerActive'] = True
            return {'RUNNING_MODAL'}
        else:
            print('Error connecting to external interface, stopping')
            return {'CANCELLED'}


    def cancel(self, context):
        print('Stopping Command Listener')
        if self._timer:
            wm = context.window_manager
            wm.event_timer_remove(self._timer)
        else:
            print('no timer')

        for src in self.cmd_sources:
            src.drop()

        bpy.context.scene['commandListenerActive'] = False
        return {'CANCELLED'}


    @classmethod
    def poll(cls, context):
        return not bpy.context.scene['commandListenerActive']
        # return True


def register():
    bpy.utils.register_class(BLCommandListener)


def unregister():
    bpy.utils.unregister_class(BLCommandListener)


def refresh():
    try:
        register()
    except Exception as E:
        print('re-registering')
        print(E)
        unregister()
        register()

# Find and load command sources like ROS
def load_cmd_sources():
    import pkg_resources

    # Command Sources must advertise their build functions
    # under this entry point group
    group = 'blender_api.command_source.build'

    entry_points = list(pkg_resources.iter_entry_points(group))
    cmdsrcs = []
    names = []
    for point in entry_points:
        try:
            build = point.load()
            node = build(commands.EvaAPI())
            cmdsrcs.append(node)
            names.append(point.name)
        except ImportError:
            print("Command Source '%s' won't build" % point.name)

    return cmdsrcs, names
