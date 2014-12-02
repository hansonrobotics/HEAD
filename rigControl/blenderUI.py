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

		runningAnimation = context.scene['animationPlaybackActive']
		runningCommand = context.scene['commandListenerActive']

		### system ###
		if runningCommand:
			text = 'Command Listener Running'
		else:
			text='Start Command Listener'
		
		col = layout.column(align=True)
		col.operator("wm.command_listener", text=text, icon='CONSOLE')
		col.operator("wm.animation_playback", icon='ARMATURE_DATA')


		### Gestures ###
		row = layout.row()
		layout.label(text="Gestures:")
		col = layout.column(align=True)
		for i, action in enumerate(bpy.data.actions):
			if "GST" in action.name:
				if i%2 == 0:
					row = col.row(align=True)

				row.operator("eva.gestures", text=action.name[4:]).evaAction = action.name

		
		###  Emotions  ###
		row = layout.row()
		layout.label(text="Emotions:")
		col = layout.column(align = True)
		col.active = runningAnimation
		for i, emo in enumerate(bpy.evaAnimationManager.bones):
			if emo.name.startswith("EMO-"):
				if i%2 == 0:
					row = col.row(align=True)
				
				row.prop(emo,'["intensity"]', text=emo.name[4:], slider=True)

		col = layout.column(align = True)
		col.active = runningAnimation
		col.operator("eva.emotions", text='Set Emotions').evaEmotions = context.scene.evaEmotions
		col.prop(context.scene, 'evaEmotions', text='')

		# row = layout.row()
		# op = row.operator('eva.debug', text='Happy')
		# op.action = 'commands.setEmotionStates({"happy":0.8},bpy.evaAnimationManager)'

		# row = layout.row()
		# op = row.operator('eva.debug', text='recoil')
		# op.action = 'commands.setEmotionStates({"recoil":0.8},bpy.evaAnimationManager)'

		### Tracking ###
		row = layout.row()
		layout.label(text="Tracking:")
		row = layout.row()
		row.active = runningAnimation
		row.prop(context.scene, 'evaFollowMouse', text='Follow Mouse')

		col = layout.column(align = True)
		col.operator('eva.tracking', text='Up').evaTrack = [0, 0, 0.1]
		row = col.row(align=True)
		row.operator('eva.tracking', text='Left').evaTrack = [0.2, 0, 0]
		row.operator('eva.tracking', text='Centre').evaTrack = [0, 0, 0]
		row.operator('eva.tracking', text='Right').evaTrack = [-0.2, 0, 0]
		col.operator('eva.tracking', text='Down').evaTrack = [0, 0, -0.1]

		
		row = layout.row()
		eva = bpy.evaAnimationManager
		bones = eva.deformObj.pose.bones
		col = layout.column(align = True)
		col.active = runningAnimation
		col.prop(bones['eye_dart_rate'], '["value"]', text='eyeDartRate', slider = True)
		col.prop(bones['eye_wander'], '["value"]', text='eyeWander', slider = True)
		col.prop(bones['blink_rate'], '["value"]', text='blinkRate', slider = True)
		col.prop(bones['blink_duration'], '["value"]', text='blinkDuration', slider = True)
		col.prop(bones['breath_rate'], '["value"]', text='breathRate', slider = True)
		col.prop(bones['breath_intensity'], '["value"]', text='breathIntensity', slider = True)

		row = layout.row()
		layout.label(text="Debug:")
		col = layout.column(align=True)
		col.operator('eva.debug', text='availableEmotionStates()').action = 'commands.availableEmotionStates()'
		col.operator('eva.debug', text='availableEmotionGestures()').action = 'commands.availableEmotionGestures()'
		col.operator('eva.debug', text='getEmotionStates()').action = 'commands.getEmotionStates(bpy.evaAnimationManager)'
		col.operator('eva.debug', text='getEmotionGestures()').action = 'commands.getEmotionGestures(bpy.evaAnimationManager)'
		col.operator('eva.debug', text='getEmotionParams()').action = 'commands.getEmotionParams(bpy.evaAnimationManager)'


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
	