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
        layout.prop(context.scene, 'maxFPS', slider = True)


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

        # speech
        col = layout.column(align=True)
        col.label(text="Speech:")
        row = col.row(align=True)
        row.operator('eva.debug', text='A-I').action = 'commands.EvaAPI().queueViseme(vis="A")'
        row.operator('eva.debug', text='E').action = 'commands.EvaAPI().queueViseme(vis="E")'
        row.operator('eva.debug', text='F-V').action = 'commands.EvaAPI().queueViseme(vis="F")'
        row.operator('eva.debug', text='Q-W').action = 'commands.EvaAPI().queueViseme(vis="W-Q")'
        row.operator('eva.debug', text='L').action = 'commands.EvaAPI().queueViseme(vis="L")'
        row = col.row(align=True)
        row.operator('eva.debug', text='C-D-G-K-N-TH').action = 'commands.EvaAPI().queueViseme(vis="C")'
        row.operator('eva.debug', text='M').action = 'commands.EvaAPI().queueViseme(vis="M")'
        row.operator('eva.debug', text='O').action = 'commands.EvaAPI().queueViseme(vis="O")'
        row.operator('eva.debug', text='U').action = 'commands.EvaAPI().queueViseme(vis="U")'

        ### Gestures ###
        row = layout.row()
        layout.label(text="Gestures:")
        col = layout.column(align=True)
        for i, action in enumerate(bpy.data.actions):
            if "GST" in action.name:
                if i%2 == 0:
                    row = col.row(align=True)

                # row.operator("eva.gestures", text=action.name[4:]).evaAction = action.name
                row.operator('eva.debug', text=action.name[4:]).action = 'commands.EvaAPI().setGesture("'+ action.name[4:] +'")'

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
        col.operator('eva.debug', text = 'Set Emotion').action =  'commands.EvaAPI().setEmotionState("'+ context.scene.evaEmotion + '")'
        col.prop(context.scene, 'evaEmotion', text='')

        # row = layout.row()
        # op = row.operator('eva.debug', text='Happy')
        # op.action = 'commands.EvaAPI().setEmotionStates({"happy":0.8},bpy.evaAnimationManager)'

        # row = layout.row()
        # op = row.operator('eva.debug', text='recoil')
        # op.action = 'commands.EvaAPI().setEmotionStates({"recoil":0.8},bpy.evaAnimationManager)'

        ### Tracking ###
        row = layout.row()
        layout.label(text="Tracking:")
        row = layout.row()
        row.active = runningAnimation
        row.prop(context.scene, 'evaFollowMouse', text='Follow Mouse')

        # Warning: X and Y Axis are rotated to accommodate Command Listener
        col = layout.column(align = True)
        col.operator('eva.debug', text='Face Up').action = 'commands.EvaAPI().setFaceTarget([1, 0, 0.35])'
        row = col.row(align=True)
        row.operator('eva.debug', text='Left').action = 'commands.EvaAPI().setFaceTarget([1, -0.5, 0])'
        row.operator('eva.debug', text='Centre').action = 'commands.EvaAPI().setFaceTarget([1, 0,0])'
        row.operator('eva.debug', text='Right').action = 'commands.EvaAPI().setFaceTarget([1, 0.5, 0])'
        col.operator('eva.debug', text='Face Down').action = 'commands.EvaAPI().setFaceTarget([1, 0, -0.35])'
        col.operator('eva.debug', text='Face Nil').action = 'commands.EvaAPI().setFaceTarget([0, 0, 0])'

        col = layout.column(align = True)
        col.operator('eva.debug', text='Gaze Up').action = 'commands.EvaAPI().setGazeTarget([1, 0, 0.3])'
        row = col.row(align=True)
        row.operator('eva.debug', text='Left').action = 'commands.EvaAPI().setGazeTarget([1, -0.5, 0])'
        row.operator('eva.debug', text='Centre').action = 'commands.EvaAPI().setGazeTarget([1, 0,0])'
        row.operator('eva.debug', text='Right').action = 'commands.EvaAPI().setGazeTarget([1, 0.5, 0])'
        col.operator('eva.debug', text='Gaze Down').action = 'commands.EvaAPI().setGazeTarget([1, 0, -0.3])'
        col.operator('eva.debug', text='Gaze Nil').action = 'commands.EvaAPI().setGazeTarget([0, 0, 0])'

        row = layout.row()
        layout.label(text="Debug:")
        col = layout.column(align=True)
        col.operator('eva.debug', text='getAPIVersion()').action = 'commands.EvaAPI().getAPIVersion()'
        col.operator('eva.debug', text='isAlive()').action = 'commands.EvaAPI().isAlive()'
        col.operator('eva.debug', text='availableEmotionStates()').action = 'commands.EvaAPI().availableEmotionStates()'
        col.operator('eva.debug', text='availableGestures()').action = 'commands.EvaAPI().availableGestures()'
        col.operator('eva.debug', text='getEmotionStates()').action = 'commands.EvaAPI().getEmotionStates()'
        col.operator('eva.debug', text='getGestures()').action = 'commands.EvaAPI().getGestures()'
        col.operator('eva.debug', text='getHeadData()').action = 'commands.EvaAPI().getHeadData()'
        col.operator('eva.debug', text='getNeckData()').action = 'commands.EvaAPI().getNeckData()'
        col.operator('eva.debug', text='getEyesData()').action = 'commands.EvaAPI().getEyesData()'
        col.operator('eva.debug', text='getFaceData()').action = 'commands.EvaAPI().getFaceData()'

class BLActuatorControl(bpy.types.Panel):
    bl_label = "Actuator Control"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_context = 'object'


    def draw(self, context):
        layout = self.layout
        if not hasattr(bpy.context.scene, 'actuators'):
            return

        # Draw UI for every actuator
        for attr in dir(bpy.context.scene.actuators):
            if not attr.startswith('ACT_'):
                continue
            bl_actuator = getattr(bpy.context.scene.actuators, attr)

            row = layout.row()
            row.label(icon='FORCE_TURBULENCE',
                text='{}:'.format(attr[4:]).capitalize().replace('_', ' '))
            col = row.column()
            col.alignment = 'RIGHT'
            col.prop(bl_actuator, 'HEAD_PARAM_enabled', text='On', toggle=True)

            # Draw UI for every parameter
            for attr in bl_actuator.parameter_order.split(';'):
                if attr.startswith('PARAM_'):
                    row = layout.row()
                    row.prop(bl_actuator, attr, slider=True)
                elif attr.startswith('IMG_'):
                    row = layout.row()
                    row.label(text='Open "{}" in image editor'.format(getattr(bl_actuator, attr)))
                    ## The two lines below would render the image straight in
                    ## the UI Panel, but the widget doesn't update properly.
                    # texture = bpy.data.textures[getattr(bl_actuator, attr)]
                    # row.template_preview(texture, show_buttons=False)

            # Separate actuators with some space
            layout.row()
            layout.row()

def register():
    bpy.utils.register_class(BLRigControl)
    bpy.utils.register_class(BLRigConsole)
    bpy.utils.register_class(BLActuatorControl)

def unregister():
    bpy.utils.unregister_class(BLRigControl)
    bpy.utils.unregister_class(BLRigConsole)
    bpy.utils.unregister_class(BLActuatorControl)

def refresh():
    try:
        register()
    except ValueError:
        unregister()
        register()
