# This module sets up the user interface in Blender that
# is used to manage the rigControl

import bpy

class BLRigControl(bpy.types.Panel):
	"""Creates a Panel in the Object properties window"""
	bl_label = "RigControl"
	bl_space_type = 'VIEW_3D'
	bl_region_type = 'UI'
	bl_context = "object"


	def draw(self, context):
		layout = self.layout
		obj = context.object

		### system ###
		row = layout.row()
		if context.scene['commandListenerActive']:
			prop = row.operator("wm.command_listener", text='Command Listener Running')
		else:
			prop = row.operator("wm.command_listener", text='Start Command Listener', icon='CONSOLE')
		
		row = layout.row()
		prop = row.operator("wm.animation_playback", icon='ARMATURE_DATA')

		### Gestures ###
		row = layout.row()
		layout.label(text="Gestures:")

		row = layout.row()
		
		for i, action in enumerate(bpy.data.actions):
			if "GST" in action.name:
				if i%2 == 1:
					row = layout.row()

				label = action.name.replace("GST-","")
				op = row.operator("eva.gestures", text=label)
				op.evaAction = action.name

		
		###  Emotions  ###
		row = layout.row()
		layout.label(text="Emotions:")

		row = layout.row()
		for emo in bpy.evaAnimationManager.bones:
			if emo.name.startswith("EMO-"):
				label = emo.name.replace("EMO-", "")
				layout.prop(emo,'["intensity"]', text=label, slider=True)
	
		row = layout.row()
		row.prop(context.scene, 'evaEmotions', text='')

		row = layout.row()
		prop = row.operator("eva.emotions", text='Test Emotion')
		prop.evaEmotions = context.scene.evaEmotions


		# row = layout.row()
		# op = row.operator('eva.debug', text='Happy')
		# op.action = 'commands.setEmotionStates({"happy":0.8},bpy.evaAnimationManager)'

		# row = layout.row()
		# op = row.operator('eva.debug', text='recoil')
		# op.action = 'commands.setEmotionStates({"recoil":0.8},bpy.evaAnimationManager)'

		### Tracking ###
		# row = layout.row()
		# layout.label(text="Tracking:")
		# row = layout.row()
		# op = row.operator('eva.tracking', text='Up')
		# op.evaTrack = [0, 0, 0.1]

		# row = layout.row()
		# op = row.operator('eva.tracking', text='Left')
		# op.evaTrack = [0.2, 0, 0]

		# op = row.operator('eva.tracking', text='Centre')
		# op.evaTrack = [0, 0, 0]

		# op = row.operator('eva.tracking', text='Right')
		# op.evaTrack = [-0.2, 0, 0]

		# row = layout.row()
		# op = row.operator('eva.tracking', text='Down')
		# op.evaTrack = [0, 0, -0.1]
		

		row = layout.row()
		layout.label(text="Debug:")
		row = layout.row()
		op = row.operator('eva.debug', text='availableEmotionStates()')
		op.action = 'commands.availableEmotionStates()'

		row = layout.row()
		op = row.operator('eva.debug', text='availableEmotionGestures()')
		op.action = 'commands.availableEmotionGestures()'

		row = layout.row()
		op = row.operator('eva.debug', text='getEmotionStates()')
		op.action = 'commands.getEmotionStates(bpy.evaAnimationManager)'

		row = layout.row()
		op = row.operator('eva.debug', text='getEmotionGestures()')
		op.action = 'commands.getEmotionGestures(bpy.evaAnimationManager)'


def register():
	bpy.utils.register_class(BLRigControl)


def unregister():
	bpy.utils.unregister_class(BLRigControl)


def refresh():
	try:
		register()
	except ValueError:
		unregister()
		register()
	