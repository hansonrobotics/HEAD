# This module sets up the user interface in Blender that
# is used to manage the rigControl.  That is, when various
# entries are clicked on the blender emotion/gesture selection
# panel, they are wired into here (BLRigControl), which then
# just turns around and calls methods from commands.py.

import bpy

class BLRigConsole(bpy.types.Panel):
	"""Creates a Panel in the Object properties window"""
	bl_label = "RigConsole"
	bl_space_type = 'VIEW_3D'
	bl_region_type = 'UI'
	bl_context = "object"

	def draw(self, context):
		layout = self.layout
		layout.prop(context.scene, 'evaFPS', slider = True)



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
		col.operator('eva.debug', text='Start Animation', icon='ARMATURE_DATA').action = 'commands.init()'


		### Gestures ###
		row = layout.row()
		layout.label(text="Gestures:")
		col = layout.column(align=True)
		for i, action in enumerate(bpy.data.actions):
			if "GST" in action.name:
				if i%2 == 0:
					row = col.row(align=True)

				# row.operator("eva.gestures", text=action.name[4:]).evaAction = action.name
				row.operator('eva.debug', text=action.name[4:]).action = 'commands.setGesture("'+ action.name[4:] +'")'


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
		# col.operator("eva.emotions", text='Set Emotions').evaEmotions = context.scene.evaEmotions
		# col.prop(context.scene, 'evaEmotions', text='')
		col.operator('eva.debug', text = 'Set Emotion').action =  'commands.setEmotionState("'+ context.scene.evaEmotion + '")'
		col.prop(context.scene, 'evaEmotion', text='')


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
		col.operator('eva.debug', text='Up').action = 'commands.setPrimaryTarget([0, 100, 100])'
		row = col.row(align=True)
		row.operator('eva.debug', text='Left').action = 'commands.setPrimaryTarget([100, 100, 0])'
		row.operator('eva.debug', text='Centre').action = 'commands.setPrimaryTarget([0, 100, 0])'
		row.operator('eva.debug', text='Right').action = 'commands.setPrimaryTarget([-100, 100, 0])'
		col.operator('eva.debug', text='Down').action = 'commands.setPrimaryTarget([0, 100, -100])'

		
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
		col.operator('eva.debug', text='getAPIVersion()').action = 'commands.getAPIVersion()'
		col.operator('eva.debug', text='isAlive()').action = 'commands.isAlive()'
		col.operator('eva.debug', text='availableEmotionStates()').action = 'commands.availableEmotionStates()'
		col.operator('eva.debug', text='availableGestures()').action = 'commands.availableGestures()'
		col.operator('eva.debug', text='getEmotionStates()').action = 'commands.getEmotionStates()'
		col.operator('eva.debug', text='getGestures()').action = 'commands.getGestures()'
		col.operator('eva.debug', text='getGestureParams()').action = 'commands.getGestureParams()'
		col.operator('eva.debug', text='getHeadData()').action = 'commands.getHeadData()'
		col.operator('eva.debug', text='getEyesData()').action = 'commands.getEyesData()'
		col.operator('eva.debug', text='getFaceData()').action = 'commands.getFaceData()'


def register():
	bpy.utils.register_class(BLRigControl)
	bpy.utils.register_class(BLRigConsole)


def unregister():
	bpy.utils.unregister_class(BLRigControl)
	bpy.utils.unregister_class(BLRigConsole)


def refresh():
	try:
		register()
	except ValueError:
		unregister()
		register()

