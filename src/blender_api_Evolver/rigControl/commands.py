# Implements the commands defined by the public API
import bpy
from mathutils import Matrix, Euler
from math import pi
from collections import OrderedDict
import logging
import pymsgbox
from random import randint

from rigAPI.rigAPI import RigAPI
from GA_methods import IGA

logger = logging.getLogger('hr.blender_api.rigcontrol.commands')
# ====================================================

def init():
    bpy.ops.wm.animation_playback()
    return 0

def getEnvironment():
    return None

def terminate():
    return 0


class EvaAPI(RigAPI):
    selected_actions = IGA.selected_actions
    initial_population = [] # initialize population for the IGA


    def __init__(self):
        pass

    # System control and information commands ===========
    def getAPIVersion(self):
        return 4

    def isAlive(self):
        return int(bpy.context.scene['animationPlaybackActive'])

    # Somatic states  --------------------------------
    # awake, asleep, drunk, dazed and confused ...
    def availableSomaStates(self):
        somaStates = []
        for state in bpy.data.actions:
            if state.name.startswith("CYC-"):
                somaStates.append(state.name[4:])
        return somaStates

    def getSomaStates(self):
        eva = bpy.evaAnimationManager
        somaStates = {}
        for cycle in eva.cyclesSet:
            magnitude = round(cycle.magnitude, 3)
            rate = round(cycle.rate, 3)
            ease_in = round(cycle.ease_in, 3)
            somaStates[cycle.name] = {'magnitude': magnitude, 'rate': rate,
                'ease_in': ease_in}
        return somaStates

    def setSomaState(self, state):
        name = 'CYC-' + state['name']
        rate = state['rate']
        magnitude = state['magnitude']
        ease_in = state['ease_in']
        bpy.evaAnimationManager.setCycle(name=name,
            rate=rate, magnitude=magnitude, ease_in=ease_in)
        return 0

    # Emotion expressions ----------------------------
    # smiling, frowning, bored ...
    def availableEmotionStates(self):
        emotionStates = []
        for emo in bpy.data.objects['control'].pose.bones:
            if emo.name.startswith('EMO-'):
                emotionStates.append(emo.name[4:])
        return emotionStates


    def getEmotionStates(self):
        eva = bpy.evaAnimationManager
        emotionStates = {}
        for emotion in eva.emotionsList:
            magnitude = round(emotion.magnitude.current, 3)
            emotionStates[emotion.name] = {'magnitude': magnitude}
        return emotionStates


    def setEmotionState(self, emotion):
        # TODO: expand arguments and update doc
        bpy.evaAnimationManager.setEmotion(eval(emotion))
        return 0


    # Gestures --------------------------------------
    # blinking, nodding, shaking...
    def availableGestures(self):
        emotionGestures = []
        for gesture in bpy.data.actions:
            if gesture.name.startswith("GST-"):
                emotionGestures.append(gesture.name[4:])
        return emotionGestures


    def getGestures(self):
        eva = bpy.evaAnimationManager
        emotionGestures = {}
        for gesture in eva.gesturesList:
            duration = round(gesture.duration*gesture.repeat - gesture.stripRef.strip_time, 3)
            magnitude = round(gesture.magnitude, 3)
            speed = round(gesture.speed, 3)
            emotionGestures[gesture.name] = {'duration': duration, \
                'magnitude': magnitude, 'speed': speed}
        return emotionGestures


    def setGesture(self, name, repeat=1, speed=1, magnitude=1.0):
        bpy.evaAnimationManager.newGesture(name='GST-'+name, \
            repeat=repeat, speed=speed, magnitude=magnitude)
        return 0

    # Genetic algorithm related functions   
    def setGesture_IGA(self, name, repeat=1, speed=1, magnitude=1.0):

        bpy.evaAnimationManager.newGesture_IGA(name='IGA-'+name, \
            repeat=repeat, speed=speed, magnitude=magnitude)
        return 0    

    # Add Initial population set to the IGA panel  
    def listPopulation_IGA(self):
        action_exists = 0
        action_names = bpy.data.scenes['Scene']['action_add_IGA'] # An example input can be... GST-amused : GST-thoughtful : GST-nod-1 : GST-shake-2
        action_list = action_names.split(' : ') #converts text input into lists
        print (action_list)
        
        #checked_list = []
        for i in range(0, len(action_list)):
            flag = 0
            action_exists = 0

            for j, gesture in enumerate(bpy.data.actions):  
                if gesture.name == action_list[i] and flag == 0:
                    previous_name = gesture.name[4:] # [4:] removing the 'GST-' prefix copies it to the previous_name variable.
                    gesture = gesture.copy()
                    gesture.name= 'IGA-' + previous_name # So, for example the new copy of 'GST-thought' will be 'IGA-thoughtful'
                    action_exists = 1
                    flag = 1
                    if gesture.name not in self.initial_population:
                        self.initial_population.append(gesture.name)
                        print("An action named "+ gesture.name + " is imported to the parent population set panel for IGA")
                               
            if action_exists == 0:
                print("An action named " + action_list[i] + " doesn't exist. Make sure it is imported!")
        print(self.initial_population)
        return self.initial_population   
    
    # Import population set to the IGA panel  
    def importPopulation_IGA(self, name, repeat=1, speed=1, magnitude=1.0):
        bpy.evaAnimationManager.newGesture_IGA(name= name, \
            repeat=repeat, speed=speed, magnitude=magnitude)
        return 0      



    def select_Facialparts(self):#, selected):

        action_list = ['bool_action1', 'bool_action2', 'bool_action3', 'bool_action4', 'bool_action5', 'bool_action6', 'bool_action7', 'bool_action8', 'bool_action9', 'bool_action10', 'bool_action11', 'bool_action12', 'bool_action13', 'bool_action14', 'bool_action15',  'bool_action16', 'bool_action17', 'bool_action18', 'bool_action19', 'bool_action20', 'bool_action21', 'bool_action22', 'bool_action23',  'bool_action24', 'bool_action25', 'bool_action26', 'bool_action27', 'bool_action28', 'bool_action29', 'bool_action30', 'bool_action31', 'bool_action32', 'bool_action33', 'bool_action34', 'bool_action35', 'bool_action36', 'bool_action38', 'bool_action39', 'bool_action40', 'bool_action41', 'bool_action42', 'bool_action43', 'bool_action44', 'bool_action45']

        '''selected = selected.split(',')
        sel_list = []
        for s in range(0, len(selected)):
            sel_list.append(selected[s][2])'''
        
        eyes = bpy.data.scenes['Scene'].bool_facialpart.bool_eyes
        mouth = bpy.data.scenes['Scene'].bool_facialpart.bool_mouth
        head = bpy.data.scenes['Scene'].bool_facialpart.bool_head


        ticked_boxes = bpy.data.scenes["Scene"].bool_actions.items()
        ticked_actions = sorted(ticked_boxes)

        ticked_actions_list = []
        ticked_boxes = bpy.data.scenes["Scene"].bool_actions.items()
        ticked_actions = sorted(ticked_boxes)
        print(ticked_actions)
        for i in range(0, len(ticked_actions)):
            if ticked_actions[i][1] == 1:
                for j in range(0, len(self.initial_population)):   
                    if ticked_actions[i][0] == action_list[j]:
                         if eyes == True:
                             if self.initial_population[j] + '_eyes' not in self.selected_actions:
                                 self.selected_actions.append(self.initial_population[j] +'_'+'eyes') 
                         if mouth == True:
                             if self.initial_population[j] + '_mouth' not in self.selected_actions:
                                 self.selected_actions.append(self.initial_population[j] +'_'+'mouth') 
                         if head == True:
                             if self.initial_population[j] + '_head' not in self.selected_actions:
                                 self.selected_actions.append(self.initial_population[j] +'_'+'head')  
                                 break
        print('selected facial part')
        print(self.selected_actions)
        bpy.data.scenes['Scene'].bool_facialpart.bool_eyes = False
        bpy.data.scenes['Scene'].bool_facialpart.bool_mouth = False
        bpy.data.scenes['Scene'].bool_facialpart.bool_head = False
        '''if len(selected) > 1:
            self.selected_actions = sel_list
            print(self.selected_actions)
            return self.selected_actions
        else:
            return self.selected_actions'''
        return 0

# A hack function: Renaming an action imported into the IGA panel would cause it to be removed from the list (though not from the blender instance).
    #def removeActions_fromIGA(self, ticked_actions):
    def removeActions_fromIGA(self):
        action_list = ['bool_action1', 'bool_action2', 'bool_action3', 'bool_action4', 'bool_action5', 'bool_action6', 'bool_action7', 'bool_action8', 'bool_action9', 'bool_action10', 'bool_action11', 'bool_action12', 'bool_action13', 'bool_action14', 'bool_action15',  'bool_action16', 'bool_action17', 'bool_action18', 'bool_action19', 'bool_action20', 'bool_action21', 'bool_action22', 'bool_action23',  'bool_action24', 'bool_action25', 'bool_action26', 'bool_action27', 'bool_action28', 'bool_action29', 'bool_action30', 'bool_action31', 'bool_action32', 'bool_action33', 'bool_action34', 'bool_action35', 'bool_action36', 'bool_action38', 'bool_action39', 'bool_action40', 'bool_action41', 'bool_action42', 'bool_action43', 'bool_action44', 'bool_action45']
        ticked_actions_list = []
        ticked_boxes = bpy.data.scenes["Scene"].bool_actions.items()
        ticked_actions = sorted(ticked_boxes)
        print (ticked_boxes)
        inc = 0
        for i in range(0, len(ticked_actions)):
            if ticked_actions[i][1] == 1:
                for j in range(0, len(self.initial_population)):
                    if ticked_actions[i][0] == action_list[j]:
                        ticked_actions_list.append(self.initial_population[j]) 
                        break

        print("tickedlist")
        print(ticked_actions_list)  
        parts = ['eyes', 'mouth', 'head']

 
        for i in range(0, len(ticked_actions_list)):   
            print('similarity found')
            for p in range(0, len(parts)):
                for action_index in range(0, len(self.selected_actions)):
                    print(len(self.selected_actions)) 
                    if str(ticked_actions_list[i]) == self.selected_actions[action_index].split('_')[0]:
                        print('similarity found2') 
                        if self.selected_actions[action_index].split('_')[1] == parts[0]:
                            self.selected_actions.remove(self.selected_actions[action_index].split('_')[0]+'_eyes')
                            #self.selected_actions.remove(str(ticked_actions_list[i])+'_eyes')
                            print(str(ticked_actions_list[i]))
                            print('eyes removed')
                            break
                        if self.selected_actions[action_index].split('_')[1] == parts[1]:
                            self.selected_actions.remove(self.selected_actions[action_index].split('_')[0]+'_mouth') 
                            #self.selected_actions.remove(str(ticked_actions_list[i])+'_mouth') 
                            print(str(ticked_actions_list[i]))
                            print('mouth removed')
                            break
                        if self.selected_actions[action_index].split('_')[1] == parts[2]:
                            self.selected_actions.remove(self.selected_actions[action_index].split('_')[0]+'_head')
                            #self.selected_actions.remove(str(ticked_actions_list[i])+'_head')
                            print(str(ticked_actions_list[i]))
                            print('head removed')
                            break         

        print('self.selected_actions')         
        print (self.selected_actions)  
                     
                

# A hack function: Renaming an action imported into the IGA panel would cause it to be removed from the Panel list (though not from the blender instance).

        for i in range(0, len(ticked_actions_list)):
            for j, action in enumerate(bpy.data.actions):
                if str(ticked_actions_list[i]) == action.name:
                    self.initial_population.remove(str(ticked_actions_list[i])) 
                    print("An action named "+ action.name + " is removed from the IGA panel by renaming it to" + "removed-" + action.name )
                    action.name = 'removed-' + action.name  
                    break
        return 0 

# A function to add/save a list of chosen actions into the current blend file. But the .blend file needs to be saved before closing in order to make the save permanent.
  
    def saveAction_IGA(self):
        
        action_exists = 0
        action_names = bpy.data.scenes['Scene'].action_save_IGA # An example input may look like... IGA-amused : IGA-thoughtful : IGA-nod-1 etc.(put in the list names of evolved actions you want to be saved.)
        action_list = action_names.split(' : ') #converts text input into lists
        print (action_list)
        
        #checked_list = []
        for i in range(0, len(action_list)):
            flag = 0
            action_exists = 0

            for j, gesture in enumerate(bpy.data.actions):  
                if gesture.name == action_list[i] and flag == 0:
                    bpy.data.actions[gesture.name].use_fake_user = True
                    action_exists = 1
                    flag = 1
            if action_exists == 0:
                print("An action named " + action_list[i] + " is not known!")

# A function to rename an action's current name to a new one   
    
    def renameAction(self):
        print(self.initial_population)
        action_exists = 0
        current_name = bpy.data.scenes['Scene'].action_currentname
        new_name = bpy.data.scenes['Scene'].action_newname
        for gesture in bpy.data.actions:
            if gesture.name == current_name:
                bpy.data.actions[gesture.name].name = new_name
                action_exists = 1
                print("An action named"+ gesture.name + "is renamed to" + new_name)
        if action_exists == 0:
            print("An action named" + current_name + "doesn't exit")

        return 0
 # calls and executes the d/t crossover methods defined in the animationManager.py based on the weight and boolean values.           
    #def childGeneration_uniform(self):
    def childGenerator(self):
        
        population  = self.initial_population

        bool_uniform = bpy.data.scenes['Scene'].bool_crossover.bool_uniform
        bool_cutandsplice = bpy.data.scenes['Scene'].bool_crossover.bool_cutandsplice
        bool_blend = bpy.data.scenes['Scene'].bool_crossover.bool_blend 
    
        weight_uniform = bpy.data.scenes['Scene'].weight_uniform
        weight_cutandsplice = bpy.data.scenes['Scene'].weight_cutandsplice
        weight_blend = bpy.data.scenes['Scene'].weight_blend
        weight_mutation = bpy.data.scenes['Scene'].weight_mutation

        popn_uniform = int((len(population)* weight_uniform))
        popn_cutandsplice = int((len(population)* weight_cutandsplice))
        popn_blend = int((len(population)* weight_blend))
       
        popn_range_first = randint(1, 2)
        popn_range_second = 0 
        popn_range_third = 3
        
        if popn_range_first == 1:
            popn_range_second = 2
        else:    
            popn_range_second = 1
        

              ### weight based expression generation logic ###

        # uniform, cut&splice and blend crossovers selected
        if bool_uniform  == True and bool_cutandsplice == True and bool_blend == True:
            if weight_uniform + weight_cutandsplice + weight_blend > 1.001:
                pymsgbox.alert("Total weight value exceeded 1", "Weight Limit")
                #print ("Total weight value exceeded limit of maximum value (1)")
            else:
                bpy.evaAnimationManager.Generator_uniform(self.initial_population, popn_uniform, popn_range_first, weight_mutation, self.selected_actions)
                weight_mutation=0
                bpy.evaAnimationManager.Generator_cutandsplice(self.initial_population, popn_cutandsplice, popn_range_second, weight_mutation, self.selected_actions)
                bpy.evaAnimationManager.Generator_blend(self.initial_population, popn_blend, popn_range_third, weight_mutation, self.selected_actions)

        # uniform and cut&splice crossovers selected
        if bool_uniform  == True and bool_cutandsplice == True and bool_blend == False:
            if weight_uniform + weight_cutandsplice > 1.001:
                pymsgbox.alert("Total weight value exceeded 1", "Weight Limit")
                #print ("Total weight value exceeded limit of maximum value (1)")
            else:
                bpy.evaAnimationManager.Generator_uniform(self.initial_population, popn_uniform, popn_range_first, weight_mutation, self.selected_actions)
                weight_mutation=0
                bpy.evaAnimationManager.Generator_cutandsplice(self.initial_population, popn_cutandsplice, popn_range_second, weight_mutation, self.selected_actions)

        # uniform and blend crossovers selected
        if bool_uniform  == True and bool_cutandsplice == False and bool_blend == True:
            if weight_uniform + weight_blend > 1.001:
                pymsgbox.alert("Total weight value exceeded 1", "Weight Limit")
                #print ("Total weight value exceeded limit of maximum value (1)")
            else:
                bpy.evaAnimationManager.Generator_uniform(self.initial_population, popn_uniform, popn_range_first, weight_mutation, self.selected_actions)
                weight_mutation=0
                bpy.evaAnimationManager.Generator_blend(self.initial_population, popn_blend, popn_range_first, weight_mutation, self.selected_actions)

        # cut&splice and blend crossovers selected
        if bool_uniform  == False and bool_cutandsplice == True and bool_blend == True:
            if weight_cutandsplice + weight_blend > 1.001:
                pymsgbox.alert("Total weight value exceeded 1", "Weight Limit")
                #print ("Total weight value exceeded limit of maximum value (1)")
            else:
                bpy.evaAnimationManager.Generator_cutandsplice(self.initial_population, popn_cutandsplice, popn_range_first, weight_mutation, self.selected_actions)
                weight_mutation=0
                bpy.evaAnimationManager.Generator_blend(self.initial_population, popn_blend, popn_range_second, weight_mutation, self.selected_actions)
       
        popn_range_full = 4
        # uniform crossover selected
        if bool_uniform  == True and bool_cutandsplice == False and bool_blend == False:
            if weight_uniform > 1.001:
                pymsgbox.alert("Total weight value exceeded 1", "Weight Limit")
                #print ("Total weight value exceeded limit of maximum value (1)")
            else:
                bpy.evaAnimationManager.Generator_uniform(self.initial_population, popn_uniform, popn_range_full, weight_mutation, self.selected_actions)

        # cut&splice crossover selected
        if bool_uniform  == False and bool_cutandsplice == True and bool_blend == False:
            if weight_cutandsplice > 1.001:
                pymsgbox.alert("Total weight value exceeded 1", "Weight Limit")
                #print ("Total weight value exceeded limit of maximum value (1)")
            else:
                bpy.evaAnimationManager.Generator_cutandsplice(self.initial_population, popn_cutandsplice, popn_range_full, weight_mutation, self.selected_actions)

        # blend crossover selected
        if bool_uniform  == False and bool_cutandsplice == False and bool_blend == True:
            if weight_blend > 1.001:
                pymsgbox.alert("Total weight value exceeded 1", "Weight Limit")
                #print ("Total weight value exceeded limit of maximum value (1)")
            else:
                
                bpy.evaAnimationManager.Generator_blend(self.initial_population, popn_blend, popn_range_full, weight_mutation, self.selected_actions)

        return 0
        

   
 # calls and executes the cut and splice based GA function defined in the animationManager.py  
    '''def childGeneration_cutandsplice(self):
        population  = self.initial_population
        weight_cutandsplice =  bpy.data.scenes['Scene'].weight_cutandsplice
        bpy.evaAnimationManager.Generator_cutandsplice(population, weight_cutandsplice)
        return 0'''


    def stopGesture(self, gestureID, smoothing):
        ## TODO
        return 0

    # Visemes --------------------------------------
    def availableVisemes(self):
        visemes = []
        for viseme in bpy.data.actions:
            if viseme.name.startswith("VIS-"):
                visemes.append(viseme.name[4:])
        return visemes


    def queueViseme(self, vis, start=0, duration=0.5, \
            rampin=0.1, rampout=0.8, magnitude=1):
        return bpy.evaAnimationManager.newViseme("VIS-"+vis, duration, \
            rampin, rampout, start)

    # Eye look-at targets ==========================
    # The coordinate system used is head-relative, in 'engineering'public_ws/src/blender_api/rigControl/commands.py:135
    # coordinates: 'x' is forward, 'y' to the left, and 'z' up.
    # Distances are measured in meters.  Origin of the coordinate
    # system is somewhere (where?) in the middle of the head.

    def setFaceTarget(self, loc):
        # Eva uses y==forward x==right. Distances in meters from
        # somewhere in the middle of the head.
        mloc = [loc[1], loc[0], loc[2]]
        bpy.evaAnimationManager.setFaceTarget(mloc)
        return 0

    def setGazeTarget(self, loc):
        mloc = [loc[1],  loc[0], loc[2]]
        bpy.evaAnimationManager.setGazeTarget(mloc)
        return 0
    # ========== procedural animations with unique parameters =============
    def setBlinkRandomly(self,interval_mean,interval_variation):
        bpy.evaAnimationManager.setBlinkRandomly(interval_mean,interval_variation)
        return 0

    def setSaccade(self,interval_mean,interval_variation,paint_scale,eye_size,eye_distance,mouth_width,mouth_height,weight_eyes,weight_mouth):
        bpy.evaAnimationManager.setSaccade(interval_mean,interval_variation,paint_scale,eye_size,eye_distance,mouth_width,mouth_height,weight_eyes,weight_mouth)
        return 0

    # ========== info dump for ROS, Should return non-blender data structures

    # Gets Head rotation quaternion in XYZ format in blender independamt
    # data structure.
    # Pitch: X (positive down, negative up)?
    # Yaw: Z (negative right to positive left)
    #
    # The bones['DEF-head'].id_data.matrix_world currently return the
    # unit matrix, and so are not really needed.

    def getHeadData(self):
        bones = bpy.evaAnimationManager.deformObj.pose.bones
        rhead = bones['DEF-head'].matrix * Matrix.Rotation(-pi/2, 4, 'X')
        rneck = bones['DEF-neck'].matrix * Matrix.Rotation(-pi/2, 4, 'X')
        rneck.invert()

        # I think this is the correct order for the neck rotations.
        q = (rneck * rhead).to_quaternion()
        # q = (rhead * rneck).to_quaternion()
        return {'x':q.x, 'y':q.y, 'z':q.z, 'w':q.w}

    # Same as head, but for the lower neck joint.
    def getNeckData(self):
        bones = bpy.evaAnimationManager.deformObj.pose.bones
        rneck = bones['DEF-neck'].matrix * Matrix.Rotation(-pi/2, 4, 'X')
        q = rneck.to_quaternion()
        return {'x':q.x, 'y':q.y, 'z':q.z, 'w':q.w}

    # Gets Eye rotation angles:
    # Pitch: down(negative) to up(positive)
    # Yaw: left (negative) to right(positive)

    def getEyesData(self):
        bones = bpy.evaAnimationManager.deformObj.pose.bones
        head = (bones['DEF-head'].id_data.matrix_world*bones['DEF-head'].matrix*Matrix.Rotation(-pi/2, 4, 'X')).to_euler()
        leye = bones['eye.L'].matrix.to_euler()
        reye = bones['eye.R'].matrix.to_euler()
        # Relative to head. Head angles are inversed.
        leye_p = leye.x + head.x
        leye_y = pi - leye.z if leye.z >= 0 else -(pi+leye.z)
        reye_p = reye.x + head.x
        reye_y = pi - reye.z if reye.z >= 0 else -(pi+reye.z)
        # Add head target
        leye_y += head.z
        reye_y += head.z
        return {'l':{'p':leye_p,'y':leye_y},'r':{'p':reye_p,'y':reye_y}}


    def getFaceData(self):
        shapekeys = OrderedDict()
        for shapekeyGroup in bpy.data.shape_keys:
            # Hardcoded to find the correct group
            if shapekeyGroup.name == 'ShapeKeys':
                for kb in shapekeyGroup.key_blocks:
                    shapekeys[kb.name] = kb.value

        # Fake the jaw shapekey from its z coordinate
        jawz = bpy.evaAnimationManager.deformObj.pose.bones['chin'].location[2]
        shapekeys['jaw'] = min(max(jawz*7.142, 0), 1)

        return shapekeys


    def setNeckRotation(self, pitch, roll):
        bpy.evaAnimationManager.deformObj.pose.bones['DEF-neck'].rotation_euler = Euler((pitch, 0, roll))

    def setParam(self, key, value):
        cmd = "%s=%s" % (str(key), str(value))
        logger.info("Run %s" % cmd)
        try:
            exec(cmd)
        except Exception as ex:
            logger.error("Error %s" % ex)
            return False
        return True

    def getParam(self, param):
        param = param.strip()
        logger.info("Get %s" % param)
        try:
            return str(eval(param))
        except Exception as ex:
            logger.error("Error %s" % ex)

    def getAnimationLength(self, animation):
        animation = "GST-"+animation
        if not animation in bpy.data.actions.keys():
            return 0
        else:
            frame_range = bpy.data.actions[animation].frame_range
            frames = 1+frame_range[1]-frame_range[0]
            return frames / bpy.context.scene.render.fps

