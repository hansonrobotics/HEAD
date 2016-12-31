# This module sets up the user interface in Blender that
# is used to manage the rigControl.  That is, when various
# entries are clicked on the blender emotion/gesture selection
# panel, they are wired into here (BLRigControl), which then
# just turns around and calls methods from commands.py.

import bpy
from .commands import EvaAPI

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



from bpy.props import (StringProperty,
                       BoolProperty,
                       IntProperty,
                       FloatProperty,
                       FloatVectorProperty,
                       EnumProperty,
                       PointerProperty,
                       )
from bpy.types import (Panel,
                       Operator,
                       AddonPreferences,
                       PropertyGroup,
                       )



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

class MySettings(PropertyGroup):

    my_bool = BoolProperty(
        name="Enable or Disable",
        description="A bool property",
        default = False)

    my_bool1 = my_bool2 = my_bool3 =  my_bool4 = my_bool5 = my_bool = my_bool6 = my_bool7 = my_bool8 = my_bool9 = my_bool10 = my_bool11 = my_bool12 = my_bool13 = my_bool14 = my_bool15 = my_bool16 =  my_bool17 = my_bool18 = my_bool19 = my_bool20 = my_bool21 = my_bool22 = my_bool23 = my_bool24 = my_bool25 = my_bool26 = my_bool27 = my_bool28 = my_bool29 = my_bool30 = my_bool31 = my_bool32 = my_bool33 = my_bool34 = my_bool35 = my_bool36 = my_bool37 = my_bool38 = my_bool39 = my_bool40 = my_bool41 = my_bool42 = my_bool43 = my_bool44 = my_bool45 = my_bool

    bool_eyes = bool_mouth = bool_head = bool_uniform = bool_cutandsplice = bool_blend = bool_action1 = bool_action2 = bool_action3 = bool_action4 = bool_action5 = bool_action6 = bool_action7 = bool_action8 = bool_action9 = bool_action10 = bool_action11 = bool_action12 = bool_action13 = bool_action14 = bool_action15 = bool_action16 = bool_action17 = bool_action18 = bool_action19 = bool_action20 = bool_action21 = bool_action22 = bool_action23 = bool_action24 = bool_action25 = bool_action26 = bool_action27 = bool_action28 = bool_action29 = bool_action30 = bool_action31 = bool_action32 = bool_action33 = bool_action34 = bool_action35 = bool_action36 = bool_action38 = bool_action39 = bool_action40 = bool_action41 = bool_action42 = bool_action43 = bool_action44 = bool_action45 =  my_bool

    bool_child1 = bool_child2 = bool_child3 = bool_child4 = bool_child5 = bool_child6 = bool_child7 = bool_child8 = bool_child9 = bool_child10 = bool_child11 = bool_child12 = bool_child13 = bool_child14 = bool_action15 = bool_child16 = bool_child17 = bool_child18 = bool_child19 = bool_child20  = bool_child21 = bool_child22 = bool_child23 = bool_child24 = bool_child25 = bool_child26 = bool_child27 = bool_child28 = bool_child29  = bool_child30 = bool_child31 = bool_child32 = bool_child33 = bool_child34 = bool_child35 = bool_child36 = bool_child37 = bool_child38 = bool_child39 =bool_action40 = bool_child41 = bool_child42 = bool_child43 = bool_child44 = bool_child45 = bool_child46 = bool_child47= bool_child48 = bool_child49 = bool_child50 = bool_action51 = bool_action52 = bool_action53 = bool_action54 = bool_action55 =  bool_action56 = bool_action57 = bool_action58 = bool_action59 = bool_action60 = bool_action61 = bool_action62 = bool_action63 = bool_action64 =  bool_action65 = bool_action66 = bool_action67 = bool_action68 = bool_action69 = bool_action70 = bool_action71 = bool_action72 =  bool_action73 = bool_action74 = bool_action75 = bool_child76 = bool_child78 = bool_child97 = bool_child80 = bool_child81=  bool_child82 = bool_child83 = bool_child84 = bool_action85 = bool_child86 = bool_child87 = bool_child88 = bool_child89 = bool_child90 =  bool_child91 = bool_child92 = bool_child93 = bool_child94 = bool_child95 = bool_child96 = bool_child97 = bool_child98 = bool_child99 =  bool_child100 = bool_child101  = bool_child102 = bool_child103 = bool_child104 = bool_action105 = bool_child106 = bool_child107 =  bool_child108 = bool_child109 =  bool_child110 = bool_child111 = bool_child112 = bool_child113 = bool_child114 = bool_child115 =  bool_child116 = bool_child117 = bool_child118 = bool_child119 = bool_action120 = bool_child121 = bool_child122 =  bool_child123 = bool_child124 = bool_child125 = bool_child126 = bool_child127 = bool_child128 = bool_child129 = bool_child130 = bool_child131 = bool_child132 = bool_child133 = bool_child134 = bool_action135 = bool_child136 = bool_child137 =  bool_child138 = bool_child139 = bool_child140 = bool_child141 = bool_child142 = bool_child143 = bool_child144 = bool_child145 =  bool_child146 = bool_child147 = bool_child148 = bool_child149 = bool_child150 = my_bool


class IGA_Panel(Panel):
    '''A UI to experiment evolution of 3D facial expressions using an Interactive Genetic Algorithm'''
    bl_idname = "IGA_panel"
    bl_label = "IGA based Facial Animation Generation"
    bl_category = "My Category"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'


    def draw(self, context):
        layout = self.layout
        obj = context.object
        scn = context.scene
        select_parent_action = scn.bool_parent_actions
        select_child_action = scn.bool_child_actions
        crossovertype = scn.bool_crossover
        facialpart = scn.bool_facialpart


        crossover_list = ['bool_uniform','bool_cutandsplice','bool_blend']
        facialpart_list = ['bool_eyes','bool_mouth','bool_head']

        #for i in range(bpy.data.actions)

        parent_list = ['bool_action1', 'bool_action2', 'bool_action3', 'bool_action4', 'bool_action5', 'bool_action6', 'bool_action7', 'bool_action8', 'bool_action9', 'bool_action10', 'bool_action11', 'bool_action12', 'bool_action13', 'bool_action14', 'bool_action15',  'bool_action16', 'bool_action17', 'bool_action18', 'bool_action19', 'bool_action20', 'bool_action21', 'bool_action22', 'bool_action23',  'bool_action24', 'bool_action25', 'bool_action26', 'bool_action27', 'bool_action28', 'bool_action29', 'bool_action30', 'bool_action31', 'bool_action32', 'bool_action33', 'bool_action34', 'bool_action35', 'bool_action36', 'bool_action38', 'bool_action39', 'bool_action40', 'bool_action41', 'bool_action42', 'bool_action43', 'bool_action44', 'bool_action45']

        child_list = ['bool_child1', 'bool_child2', 'bool_child3', 'bool_child4', 'bool_child5', 'bool_child6', 'bool_child7', 'bool_child8', 'bool_child9', 'bool_child10', 'bool_child11', 'bool_child12', 'bool_child13', 'bool_child14', 'bool_action15',  'bool_child16', 'bool_child17', 'bool_child18', 'bool_child19', 'bool_child20', 'bool_child21', 'bool_child22', 'bool_child23',  'bool_child24', 'bool_child25', 'bool_child26', 'bool_child27', 'bool_child28', 'bool_child29', 'bool_child30', 'bool_child31', 'bool_child32', 'bool_child33', 'bool_child34', 'bool_child35', 'bool_child36', 'bool_child37', 'bool_child38', 'bool_child39', 'bool_action40',  'bool_child41', 'bool_child42', 'bool_child43', 'bool_child44', 'bool_child45', 'bool_child46', 'bool_child47', 'bool_child48',  'bool_child49', 'bool_child50', 'bool_action51', 'bool_action52', 'bool_action53', 'bool_action54', 'bool_action55', 'bool_action56', 'bool_action57', 'bool_action58', 'bool_action59', 'bool_action60', 'bool_action61', 'bool_action62', 'bool_action63', 'bool_action64', 'bool_action65',  'bool_action66', 'bool_action67', 'bool_action68', 'bool_action69', 'bool_action70', 'bool_action71', 'bool_action72', 'bool_action73',  'bool_action74', 'bool_action75', 'bool_child76', 'bool_child78', 'bool_child97', 'bool_child80', 'bool_child81', 'bool_child82', 'bool_child83', 'bool_child84', 'bool_action85',  'bool_child86', 'bool_child87', 'bool_child88', 'bool_child89', 'bool_child90', 'bool_child91', 'bool_child92', 'bool_child93',  'bool_child94', 'bool_child95', 'bool_child96', 'bool_child97', 'bool_child98', 'bool_child99', 'bool_child100', 'bool_child101', 'bool_child102', 'bool_child103', 'bool_child104', 'bool_action105',  'bool_child106', 'bool_child107', 'bool_child108', 'bool_child109', 'bool_child110', 'bool_child111', 'bool_child112', 'bool_child113',  'bool_child114', 'bool_child115', 'bool_child116', 'bool_child117', 'bool_child118', 'bool_child119', 'bool_child120', 'bool_action121',  'bool_child122', 'bool_child123', 'bool_child124', 'bool_child125', 'bool_child126', 'bool_child127', 'bool_child128', 'bool_child129',  'bool_child130', 'bool_child131', 'bool_child132', 'bool_child133', 'bool_child134', 'bool_child135', 'bool_child136', 'bool_action137',  'bool_child138', 'bool_child139', 'bool_child140', 'bool_child141', 'bool_child142', 'bool_child143', 'bool_child144', 'bool_child145',  'bool_child146', 'bool_child147', 'bool_child148',  'bool_child149', 'bool_child150']  

        
        
        ### Gestures as set of expression inputs for the IGA based Expression Evolver ###
        inc = 0
        row = layout.row()
        layout.label(text="Import Initial Population for IGA:")
        col = layout.column(align=True)

        
        act_add = 'action_add_IGA'
        
        col.prop(context.scene, act_add, text = '')  
        row = layout.row()
        row.operator('eva.debug', text="Import").action = 'commands.EvaAPI().listPopulation_IGA()'
        popn_set = EvaAPI().initial_population

        col = layout.column(align=True)
        loop = -1
        
        for n in range(0, len(popn_set)):
            for i, action in enumerate(bpy.data.actions):
                if popn_set[n] == action.name:
                    loop = loop+1
                    if inc % 2 == 0:
                        row = col.row(align=True)
                    row.prop(select_parent_action, parent_list[loop], text = '')
                    row.operator('eva.debug', text=action.name).action ='commands.EvaAPI().importPopulation_IGA("'+ action.name +'")'
                    rate_var = 'rate' + str(inc+1)
                    row.prop(context.scene, rate_var, text='rate value')
                    inc = inc + 1
                    break


        # insert the UI for the all the crossovers used
        col = layout.column()
        layout.label(text="Select Crossovers and weights:")
        #col = layout.column()

        col = layout.column(align=True)
        row = layout.row(align=True)   
        row.prop(crossovertype, crossover_list[0], text='Uniform')         
        row.prop(context.scene, "weight_uniform", text= '')


        col = layout.column(align=True)
        row = layout.row(align=True)            
        row.prop(crossovertype, crossover_list[1], text='Cut & Splice')
        row.prop(context.scene, "weight_cutandsplice", text= '')

        col = layout.column(align=True)
        row = layout.row(align=True)            
        row.prop(crossovertype, crossover_list[2], text='Blend')
        row.prop(context.scene, "weight_blend", text= '')

        col = layout.column(align=True)
        row = layout.row(align=True)  
        row.label(text="Mutation Weight")           
        row.prop(context.scene, "weight_mutation", text='weight')
        
        col = layout.column()
        row = layout.row(align=True)
        row.label(text="Select Parts of Parents to Keep:")
        row = layout.row(align=True)
        row.prop(facialpart, facialpart_list[0], text='Eyes')
        row.prop(facialpart, facialpart_list[1], text ='Mouth')       
        row.prop(facialpart, facialpart_list[2], text ='Head')
        
        #'Keep Parts of selected Actions' buttons keeps a list of selected parts (eyes, mouth or/and head) of parent expressions
        row = layout.row()
        row.operator('eva.debug', text = 'Keep Parts').action = 'commands.EvaAPI().select_Facialparts()'
        # 'Remove' button removes list of ticked actions
        act_remove = 'action_remove'
        row.operator('eva.debug', text = 'Remove Actions').action = 'commands.EvaAPI().removeActions_fromIGA()'
        
        #row.operator('eva.debug', text = 'Keep Parts of selected Actions').action = 'commands.EvaAPI().select_Facialparts("'+str(EvaAPI.selected_actions1)+'")'
        

        # A list of actions ticked
        '''ticked_actions_list = []
        ticked_actions = bpy.data.scenes["Scene"].bool_actions.items()
        ticked_actions = sorted(ticked_actions)
        print (ticked_actions)
        for i in range(0, len(ticked_actions)):
            for j in range(0, len(parent_list)):
                if ticked_actions[i][1] == 1:
                    if ticked_actions[i][0] == parent_list[j]:
                         ticked_actions_list.append(popn_set[j]) 
                         break'''
                # sort ticked textboxes' order
                

                #for j in range(0, len(popn_set)):
                #if ticked_actions[i][0] == parent_list[i]: # compares the name of the ticked bool_action from the list available actions textboxes
                #ticked_actions_list.append(popn_set[i])               
        #print ('List of ticked_actions')
        #print (ticked_actions_list)
                            

        layout1 = self.layout
        col = layout1.column() 
        row = layout1.row()
        #row.prop(context.scene, act_add, text='Action to be Added')
        #row = layout.row()
        col = layout.column(align = True)
        col.operator('eva.debug', text = 'Generate').action =  'commands.EvaAPI().childGenerator()'

        #row = layout.row()
        #row.prop(context.scene, act_remove, text='Enter Expression name:')
        #row = layout.row()

        #row.operator('eva.debug', text = 'Remove Action').action = 'commands.EvaAPI().removeActions_fromIGA('+str(ticked_actions_list)+')'
       
        row = layout.row()
        layout.label(text="IGA based Generated Expressions:")  
        col = layout.column(align=True)
        counter = -1
        for i, action in enumerate(bpy.data.actions):
            if "IGA-child" in action.name:
                counter  = counter + 1
                if i % 2 == 0:
                    row = col.row(align=True)
                row.prop(select_child_action, child_list[counter], text = '')
                row.operator('eva.debug', text=action.name[4:]).action = 'commands.EvaAPI().setGesture_IGA("'+ action.name[4:] +'")'

        # save selected animations
        row = layout.row()
        col = layout.column()
        #act_save = 'action_save_IGA'
        #col.prop(context.scene, act_save, text = '')  
        row = layout.row()
        row.operator('eva.debug', text='Save actions').action = 'commands.EvaAPI().saveAction_IGA()'

        
        #layout.prop(mytool, "my_int", text="Integer Property")
        #layout.prop(mytool, "my_float", text="Float Property")

        
#Interface for the user to rename actions to new names (metadata)
class Panel_rename_action(bpy.types.Panel):
    bl_idname = "My_panel_rename_actions"
    bl_label = "Rename actions"
    bl_category = "Rename actions category"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    def draw(self, context):
        layout = self.layout
        col = layout.column()        
        inc=1
        layout.label(text="Rename Existing Actions:")
        row = layout.row()
        action_current_name = 'action_currentname'
        action_new_name = 'action_newname'
        row.prop(context.scene, action_current_name, text='Current Name')
        row = layout.row()
        row.prop(context.scene, action_new_name, text='New Name')
        row = layout.row()
        row.operator('eva.debug', text = 'Rename action').action = 'commands.EvaAPI().renameAction()'






def register():
    bpy.utils.register_class(BLRigControl)
    bpy.utils.register_class(BLRigConsole)
    bpy.utils.register_class(BLActuatorControl)
    bpy.utils.register_class(MySettings)
    bpy.utils.register_class(IGA_Panel)
    bpy.utils.register_class(Panel_rename_action)
    bpy.types.Scene.bool_facialpart = PointerProperty(type=MySettings)
    bpy.types.Scene.bool_crossover = PointerProperty(type=MySettings)
    bpy.types.Scene.bool_parent_actions = PointerProperty(type=MySettings)
    bpy.types.Scene.bool_child_actions = PointerProperty(type=MySettings)

def unregister():
    bpy.utils.unregister_class(BLRigControl)
    bpy.utils.unregister_class(BLRigConsole)
    bpy.utils.unregister_class(BLActuatorControl)
    bpy.utils.unregister_class(MySettings)
    bpy.utils.register_class(IGA_Panel)
    bpy.utils.unregister_class(Panel_rename_action)
    del bpy.types.Scene.bool_facialpart
    del bpy.types.Scene.bool_crossover
    del bpy.types.Scene.bool_parent_actions
    del bpy.types.Scene.bool_child_actions

def refresh():
    try:
        register()
    except ValueError:
        unregister()
        register()

