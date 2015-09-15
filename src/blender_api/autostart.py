#
# Autostart the blender animation.
#
# Run this as:
#    blender -y ./Eva269.blend -P ./autostart.py
#
import bpy
# Start Timer
bpy.ops.wm.global_timer()
# Starts Command Listener
bpy.ops.wm.command_listener()
# Starts animation manager
bpy.ops.wm.animation_playback()
