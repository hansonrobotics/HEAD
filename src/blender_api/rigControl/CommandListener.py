# This module sets up a modal operator in Blender to act
# as the command listener for external events

debug = True

import bpy
import pprint
from . import commands
from rigAPI import CommandSource
import logging

import imp
imp.reload(commands)

logger = logging.getLogger('hr.blender_api.rigcontrol.commandlistener')

class BLCommandListener(bpy.types.Operator):
    """Listens for external commands"""
    bl_label = "Command Listener"
    bl_idname = 'wm.command_listener'

    cmd_sources = None
    bpy.types.Scene.commandListenerActive = bpy.props.BoolProperty( name = "commandListenerActive", default=False)
    bpy.context.scene['commandListenerActive'] = False

    def modal(self, context, event):
        if debug and event.type in {'ESC'}:
            return self.cancel(context)

        if debug and event.type == 'TIMER' and bpy.context.scene['commandListenerActive']:
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

        return {'PASS_THROUGH'}


    def execute(self, context):
        if self.poll(context):
            logger.info('Starting Command Listener')

            # Load cmd sources on first press of the button
            if self.cmd_sources == None:
                type(self).cmd_sources, names = load_cmd_sources()
                if len(self.cmd_sources) > 0:
                    logger.info("Command Source '%s' loaded" % ', '.join(names))
                else:
                    logger.warn('No Command Sources found')
            if not bpy.context.scene['globalTimerStarted']:
                bpy.ops.wm.global_timer()
            success = True
            for src in self.cmd_sources:
                src.init()
                success = src.push()

            if success:
                wm = context.window_manager
                wm.modal_handler_add(self)
                bpy.context.scene['commandListenerActive'] = True
                return {'RUNNING_MODAL'}
            else:
                logger.error('Error connecting to external interface, stopping')
                return {'CANCELLED'}

    def cancel(self, context):
        logger.info('Stopping Command Listener')

        for src in self.cmd_sources:
            src.drop()

        bpy.context.scene['commandListenerActive'] = False
        return {'CANCELLED'}


    @classmethod
    def poll(cls, context):
        return not bpy.context.scene['commandListenerActive']


def register():
    bpy.utils.register_class(BLCommandListener)


def unregister():
    bpy.utils.unregister_class(BLCommandListener)


def refresh():
    try:
        register()
    except Exception as E:
        logger.info('re-registering')
        logger.warn(E)
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
            logger.error("Command Source '%s' won't build" % point.name)

    return cmdsrcs, names
