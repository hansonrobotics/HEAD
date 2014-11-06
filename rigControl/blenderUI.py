# This module controls the user interface in Blender that
# is used to manage the rigControl

import bpy
import pdb

class BLRigControl(bpy.types.Panel):
	"""Creates a Panel in the Object properties window"""
	bl_label = "RigControl"
	bl_space_type = 'VIEW_3D'
	bl_region_type = 'UI'
	bl_context = "object"


	# bpy.types.Scene.commandListenerActive = bpy.props.BoolProperty( name = "commandListenerActive", default=False)

	def draw(self, context):
		layout = self.layout
		obj = context.object

		row = layout.row()
		if bpy.context.scene['commandListenerActive']:
			prop = row.operator("wm.command_listener", text='Command Listener Running')
		else:
			prop = row.operator("wm.command_listener", text='Start Command Listener', icon='CONSOLE')
		

		row = layout.row()
		layout.label(text="Gestures:")

		row = layout.row()
		prop = row.operator("wm.animation_playback", icon='ARMATURE_DATA')
		row = layout.row()



		for i, action in enumerate(bpy.data.actions):
			if "GST" in action.name:
				if i%2 == 1:
					row = layout.row()

				label = action.name.replace("GST-","")
				op = row.operator("eva.gestures", text=label)
				op.evaAction = action.name

		

		row = layout.row()
		layout.label(text="Emotions:")


		row = layout.row()
		prop = row.operator("eva.emotions", text='Test Emotion')
		prop.evaEmotions = ''


		row = layout.row()
		prop = row.operator("eva.emotions", text='Reset Emotion')
		prop.evaEmotions = 'reset'

		
		row = layout.row()
		layout.label(text="Tracking:")
		row = layout.row()
		op = row.operator('eva.tracking', text='Up')
		op.evaTrack = [0, 0, 0.1]

		row = layout.row()
		op = row.operator('eva.tracking', text='Left')
		op.evaTrack = [-0.2, 0, 0]

		op = row.operator('eva.tracking', text='Centre')
		op.evaTrack = [0, 0, 0]

		op = row.operator('eva.tracking', text='Right')
		op.evaTrack = [0.2, 0, 0]

		row = layout.row()
		op = row.operator('eva.tracking', text='Down')
		op.evaTrack = [0, 0, -0.1]

		# row = layout.row()
		# row.prop(context.scene, 'evaTrackingTarget', text='')


		

def register():
	bpy.utils.register_class(BLRigControl)


def unregister():
	bpy.utils.unregister_class(BLRigControl)


def refresh():
	try:
		register()
	except ValueError:
		print('Re-registering')
		unregister()
		register()
	