#
# Autostart the blender animation.
#
# Run this as:
#    blender -y ./Eva269.blend -P ./autostart.py
#
import bpy

bpy.ops.wm.command_listener()

from rigControl import commands
commands.init()
