import math
import bpy
import random
from mathutils import Vector

class IGA:
    selected_actions = []

    def childGenerator_uniformIGA(self, popn, popn_uniform, popn_range, weight_mutation, selected_group):
        print("uniform population:")
        print(popn_uniform)

    
    # An array that holds list of the rate values as inserted by the user. Default rate value set to 1 as defined in function GA_register().
        rates_scene = [bpy.data.scenes['Scene'].rate1, bpy.data.scenes['Scene'].rate2, bpy.data.scenes['Scene'].rate3, bpy.data.scenes['Scene'].rate4, bpy.data.scenes['Scene'].rate5, bpy.data.scenes['Scene'].rate6, bpy.data.scenes['Scene'].rate7, bpy.data.scenes['Scene'].rate8, bpy.data.scenes['Scene'].rate9, bpy.data.scenes['Scene'].rate10, bpy.data.scenes['Scene'].rate11, bpy.data.scenes['Scene'].rate12, bpy.data.scenes['Scene'].rate13, bpy.data.scenes['Scene'].rate14, bpy.data.scenes['Scene'].rate15, bpy.data.scenes['Scene'].rate16, bpy.data.scenes['Scene'].rate17, bpy.data.scenes['Scene'].rate18, bpy.data.scenes['Scene'].rate19, bpy.data.scenes['Scene'].rate20, bpy.data.scenes['Scene'].rate21, bpy.data.scenes['Scene'].rate22, bpy.data.scenes['Scene'].rate23, bpy.data.scenes['Scene'].rate24, bpy.data.scenes['Scene'].rate25, bpy.data.scenes['Scene'].rate26, bpy.data.scenes['Scene'].rate27, bpy.data.scenes['Scene'].rate28, bpy.data.scenes['Scene'].rate29, bpy.data.scenes['Scene'].rate30, bpy.data.scenes['Scene'].rate31, bpy.data.scenes['Scene'].rate32, bpy.data.scenes['Scene'].rate33, bpy.data.scenes['Scene'].rate34, bpy.data.scenes['Scene'].rate35, bpy.data.scenes['Scene'].rate36]


    #Put the rates_scene array (list of values put by users into the rate values textboxes; default set to 1) into a dictionary 'rates_var'.
        rates_var =[]
        for i in range(0, len(rates_scene)):
            rates_var.append(rates_scene[i])


######################################################################################
#Define a function here to insert/load default list of expressions to be imported if the user doesn't want to import.
######################################################################################


#Expressions imported into the IGA panel will be copied from 'popn' list to 'expression_list'.
        expression_list = []
        for i, action in enumerate(bpy.data.actions):
        #if action.name.startswith("GST-") and "child" not in action.name and "GST-all" not in action.name:
            for exp_name in popn:
                if action.name == exp_name:
                    expression_list.append(action)
        print ("IGA expression_list")
        print (expression_list)
     



#Here save/categorize the set of actions to be bred/crossed-over and/or to be mutated in to the arrays 'expression_rated_array' and 'action_mutate' respectively.
        expression_rated_array = []
        action_mutate = []

        for i in range(0, len(expression_list)):
            exp = expression_list[i]
            if rates_var[i] > 0:
                expression_rated_array.append(exp)
            else:
                action_mutate.append(exp)

        print ('List of actions to be cross-overed')
        print (expression_rated_array)

        print('List of actions to be mutated')
        print(action_mutate)



##################################################################
                 ### Mutation Algorithm ###
##################################################################

#################################################################################################################################################
#mutation function; we mutate actions that are rated lowest(with value 0) by the user.
# In genetic algorithm the mutation rate is adviced to be very small. Heeding to this idea,here, mutation level
# of an action which is provided for mutation(an action rated 0 by the user) is kept to as low as in average to 1/8 of the total fcurves of
# the give action; assuming that random number generator has equal distribuion.
# Besides to reduce an exaggerated effect of mutation on the selected fcurves, their keyframes' mutation are limited to random values b/n 0 and 0.02. If the mutated actions tend to be very small then you can increase mutation by narrowing the random number range in 'random_num' and also you can start increasing random range of 'val' and see the effects.
#################################################################################################################################################

        if weight_mutation > 0:
            a = 0
            while a < len(action_mutate):
                for i in range(0, len(action_mutate[a].fcurves.items())):
                    random_num = random.randint(1,8)
                    control_points = len(action_mutate[a].fcurves[i].keyframe_points.items())
                    ii = 0
                    while ii < control_points:
                        if random_num == 2:
                            val = random.uniform(0, float(weight_mutation))
                            keyframe = action_mutate[a].fcurves[i].keyframe_points[ii].co[1] = val
                            mutated_keyframes = action_mutate[a].fcurves[i].keyframe_points[ii].co
                        #print ('mutated_keyframes')
                        #print (action_mutate[a].fcurves[i].group.name)
                        ii += 1
                    print(action_mutate[a])
                a += 1

        ##########################################################
        ### Algorithm to Keep Interesting Sub-Parts (Head, Eyes and Mouth) from the Parent Facial Expressions to the Next Generation ###
        ##########################################################

        print("selected_group")
        print(selected_group)

        # Neck is included within the 'head_fcurves but since changes in values seem very sensative, the values have to be limited within limited ranges'
        head_fcurves = ['pose.bones["head"].location', 'pose.bones["head-target"].location', 'pose.bones["head"].rotation_euler', 'pose.bones["DEF-neck"].rotation_euler']
        mouth_fcurves = ['pose.bones["mouth_D_L"].location', 'pose.bones["mouth_D_R"].location', 'pose.bones["mouth_U_L"].location', 'pose.bones["mouth_U_R"].location', 'pose.bones["mouth_C_R"].location', 'pose.bones["mouth_C_L"].location', 'pose.bones["lip_C_L"].location', 'pose.bones["lip_D"].location', 'pose.bones["lip_D_L"].location', 'pose.bones["lip_D_R"].location', 'pose.bones["lip_U_L"].location', 'pose.bones["chin"].location', 'pose.bones["lip_U"].location', 'pose.bones["lip_U_R"].location']
        eyes_fcurves = ['pose.bones["blink_rate"].location', 'pose.bones["eyelid_LO_R"].location', 'pose.bones["eyelid_LO_L"].location', 'pose.bones["eyelid_blink_UP_R"].location', 'pose.bones["eyelid_U_R"].location', 'pose.bones["eyelid_U_L"].location', 'pose.bones["eye_L"].location', 'pose.bones["eye_R"].location']




        ###########################################################
        ### the very start of crossover function ###
        ###########################################################
    
        # use a reversed population as a starting population if the random generator chose the popn_range to be 2.(The second part)
        if popn_range == 2:
            expression_rated_array = list(reversed(expression_rated_array))


##############################################################################################################################################
# The block below copies a parent action (into 'copied_gesture'). So the first children generated at the children panel will be the parents
# then the during subsequent computation the children will be copied into the children storage panel in the UI.
##############################################################################################################################################
        
        # Only the parents that are included in the Uniform breeding will be copied to the child panel.
        expression_list_copy  = expression_rated_array
        popn_uniform = popn_uniform - len(action_mutate)
        copied_gesture = []
        for i in range(0, popn_uniform):
            copied_gesture.append(expression_list_copy[i].copy())
       
        #just initialize a copied expression with a '0' appended to its name. This is useful just till the first generation (Since no expression are available with a generation number appended to their names before the 1st generation)
        if popn_uniform > 0:
            copied_gesture[0].name = expression_list_copy[0].name + " " + str(0)
        else:
            print("Low total number of expressions for the uniform crossover:")
            print(popn_uniform)
    ### counts the the number of gestures with character 'child' in their name. This count helps in determining the generation number of a child gesture ###
        max_gen = 1
        exp_gen = ''
        for child_gesture in bpy.data.actions:
            if "IGA-child" in child_gesture.name:
                if len(child_gesture.name.split(" ")) > 1:
                    exp_gen = float(child_gesture.name.split(" ")[1]) # to handle string numbers that contain '.'
                    exp_gen = int(exp_gen)
                    if max_gen < exp_gen:
                        max_gen = exp_gen
            
        for j in range(0, len(copied_gesture)):
            copied_gesture[j].name = "IGA-child-" + expression_list_copy[j].name[4:] + " " +  str(int(max_gen+1))
######################################################################################################################




        a = 0
        action_counter = -1
        #while a < len(expression_rated_array)-1 and action_counter < 10:
        while a < popn_uniform-1:
            action_counter += 1
        #print ('expression')
        #print (len(array_actions_rated))
            print ("uniform expressions")
            print (expression_rated_array[a])
            print (expression_rated_array[a+1])
            f = 0
            c = 2



        #if action_counter < 33:# determines the maximum number of children that can be generated maximum 34 (since action_counter begins at -1) children per generation
        #gp = expression_rated_array[a].fcurves[i].group.name
        #for i in range(0, len(expression_rated_array[a].fcurves.items())):

        ##########################################################################################################################################################
        # 'jump_one_curve' is a variable that indicates the starting value of fcurve of an action to be skipped during the cross-over algorithm.
        # It's randomly assigned either 0 or 1. This random initialization can create two variations of children of the same set of parents
        # bred during different times. The reason is the fcurve to be jumped(based on the principle of uniform-based genetic algorithm) can be either the even one
        # fcurves of the first parent(assuming fcurves satisfy the data_path and array_index similarity condition between the two parents) if jump_one_curve is '2'
        # or the odd numbered fcurves of the first parent if jump_one_curve.
        ##########################################################################################################################################################

            jump_one_curve = random.randint(2,3)

            count_ticked_per_emotion = []
            ticked_fcurves = []
            for selected in range(0, len(selected_group)):
                selected_gp = selected_group[selected].split("_")
                if expression_rated_array[a].name == selected_gp[0]: #selected_gp[0] is the first part(the name) of the facial expression
                    count_ticked_per_emotion.append(selected_gp[1]) # store the second part(such as head, eyes or mouth) from the expression


# Here compute the length of count_ticked_per_emotion(values can range from 0 (when non is ticked) to 3 (when all three are ticked))

       # print("count_ticked_per_facial expression")
       #print(count_ticked_per_emotion)

            if len(count_ticked_per_emotion) >= 1:   
             #multi checkbox selection per emotion
                for tick_no in range(0, len(count_ticked_per_emotion)):

                    if count_ticked_per_emotion[tick_no] == 'head':
                        for head in range(0, len(head_fcurves)):
                            ticked_fcurves.append(head_fcurves[head])
                    if count_ticked_per_emotion[tick_no] == 'mouth':
                        for mouth in range(0, len(mouth_fcurves)):
                            ticked_fcurves.append(mouth_fcurves[mouth])
                    if count_ticked_per_emotion[tick_no] == 'eyes':
                        for eyes in range(0, len(eyes_fcurves)):
                            ticked_fcurves.append(eyes_fcurves[eyes])


  # Group of fcurves that are ticked are supposed to be kept during the next children generation so the first block of for-loop below jumps/skips all the ticked fcurves during breeding
            for i in range(0, len(expression_rated_array[a].fcurves.items())):
                checkbox_ticked = 0
        # ticked_fcurves contains all the fcurves within the ticked checkboxes
                for ticked in range(0, len(ticked_fcurves)):
                    if expression_rated_array[a].fcurves[i].data_path == ticked_fcurves[ticked]:
               # print ("iterate next")
               # print (ticked_fcurves[ticked])
               # print ("expression_rated_array[a].fcurves[i].data_path")
               # print (expression_rated_array[a].fcurves[i].data_path)
                        checkbox_ticked = 1

                if checkbox_ticked == 0:
                    for j in range(0, len(expression_rated_array[a+1].fcurves.items())):
                        datapath_1 = expression_rated_array[a].fcurves[i].data_path
                        arrayindex_1 = expression_rated_array[a].fcurves[i].array_index

                        datapath_2 = expression_rated_array[a+1].fcurves[j].data_path
                        arrayindex_2 = expression_rated_array[a+1].fcurves[j].array_index

# A condition that checks if given fcurves of both parents match and if 'jump_one_curve' is even it replaces the fcurves of the first parent
# is replaced by similar fcurve from the second parent, to constitute the child action. The initial value of the 'jump_one_curve' is randomly
# initialized above either to 2 or 3.

#
 # As an experiment idea, the 'jump_one_curve' can be made to be generated randomly continuously which would drive the chidren generation to #be very much random.

                        if datapath_1 == datapath_2 and arrayindex_1 == arrayindex_2 and jump_one_curve % 2 == 0:

                            jump_one_curve += 1

                   ##############################################################################################################################################
 ## Generating the 'jump_one_curve' randomly can give result to strange children actions as a result of the randomized jumping/the way to
 ## jump between fcurves either to be replaced or skipped. The current style is once it starts replacing the odd fcurves of the first parent
 ## by the second parent it continues doing so for the full set of fcurves between the two parents which constitute the child action.
 ## So you can imagine if the way we replace/skip fcurves b/n two parents in order to build a child is very random. Of course the result can be
 ## a strange action or a can also be very one-sided (a child action inclined in similarity to one of the parents). But if you think it is
 ## worth experimenting then comment the above 'jump_one_curve' and uncomment the 'jump_one_curve' statement below.
                   ##############################################################################################################################################

                   # jump_one_curve = random.randint(2,3)

                            c = control_points = len(expression_rated_array[a].fcurves[i].keyframe_points.items())-1
                            b = control_points_2 = len(expression_rated_array[a+1].fcurves[j].keyframe_points.items())-1


                    # Checks the condition that the keyframes,are to be bred, of the given fcurve in the first parent are more in number
                    # than the keyframes to of the same fcurve in the second parents, and then removes the excess keyframes in the first
                    # parent
                            if control_points - control_points_2 > 0:
                                while c > b:
                                    expression_rated_array[a].fcurves[i].keyframe_points.remove(expression_rated_array[a].fcurves[i].keyframe_points[c])
                                    c -= 1
                   # Check the condition if the first parent's keyframes of the given fcurve are less than the second parent. If such is the
                   # case then add keyframe to the first parent to match the number keyframes of the given fcurve of the second parent.
                            elif control_points - control_points_2 < 0:
                                cont_val =  control_points_2 - control_points
                                expression_rated_array[a].fcurves[i].keyframe_points.add(cont_val)

                            control_points_new = len(expression_rated_array[a].fcurves[i].keyframe_points.items())
                    #control_points_2 = len(expression_rated_array[a+1].fcurves[j].keyframe_points.items())

                    # replace the selected
                            ii = 0
                            while ii < control_points_new:
                                for x in range(0, len(head_fcurves)):
                                    if expression_rated_array[a].fcurves[i].data_path == head_fcurves[x]:
                                        val = expression_rated_array[a+1].fcurves[j].keyframe_points[ii].co[1]
                                        keyframe = expression_rated_array[a+1].fcurves[j].keyframe_points[ii].co[0]
                                        expression_rated_array[a].fcurves[i].keyframe_points[ii].co = keyframe, val
                                        ee = expression_rated_array[a].fcurves[i].keyframe_points[ii].co
                                   # print(expression_rated_array[a].fcurves[i].data_path)
                                   # print(ee)
                                   # print(expression_rated_array[a+1].fcurves[j].data_path)
                                   # print(expression_rated_array[a+1].fcurves[j].keyframe_points[ii].co)
                                   # print ("true")
                           # ii += 1
                               #if c % 2 == 0:
                            #val = random.uniform(0, 0.4)
                                keyframe = expression_rated_array[a+1].fcurves[j].keyframe_points[ii].co[0]
                                val = expression_rated_array[a+1].fcurves[j].keyframe_points[ii].co[1]
                                expression_rated_array[a].fcurves[i].keyframe_points[ii].co = keyframe, val
                                ee = expression_rated_array[a].fcurves[i].keyframe_points[ii].co
                           # print(expression_rated_array[a].fcurves[i].data_path)
                           # print(ee)
                           # print(expression_rated_array[a+1].fcurves[j].data_path)
                           # print(expression_rated_array[a+1].fcurves[j].keyframe_points[ii].co)
                           # print ("true")
                                ii += 1
                # An elif condition to increment 'jump_one_curve' when replacing of fcurve from the second parent is skipped.
                        elif datapath_1 == datapath_2 and arrayindex_1 == arrayindex_2 and jump_one_curve % 2 == 1:
                            jump_one_curve += 1

                 ### for randomized generation of 'jump_one_curve'###
                  # jump_one_curve = random.randint(2,3)

            #i += 1
            a += 1 
        print("action_counter")
        print(action_counter)
     
        # Reset the list of selected parts of expressions after a generation             
        for s in range(0, len(self.selected_actions)):
            for sel in range(0, len(self.selected_actions)): 
                self.selected_actions.remove(self.selected_actions[sel])
                break




    #******************************************************************************************************************************


    def childGenerator_cutandspliceIGA(self, popn, popn_cutandsplice, popn_range, weight_mutation):
        print("cutandsplice population:")
        print(popn_cutandsplice)
        
            # an array that holds list of the rate values as defined in the blenderPlayback.py
        rates_scene = [bpy.data.scenes['Scene'].rate1, bpy.data.scenes['Scene'].rate2, bpy.data.scenes['Scene'].rate3, bpy.data.scenes['Scene'].rate4, bpy.data.scenes['Scene'].rate5, bpy.data.scenes['Scene'].rate6, bpy.data.scenes['Scene'].rate7, bpy.data.scenes['Scene'].rate8, bpy.data.scenes['Scene'].rate9, bpy.data.scenes['Scene'].rate10, bpy.data.scenes['Scene'].rate11, bpy.data.scenes['Scene'].rate12, bpy.data.scenes['Scene'].rate13, bpy.data.scenes['Scene'].rate14, bpy.data.scenes['Scene'].rate15, bpy.data.scenes['Scene'].rate16, bpy.data.scenes['Scene'].rate17, bpy.data.scenes['Scene'].rate18, bpy.data.scenes['Scene'].rate19, bpy.data.scenes['Scene'].rate20, bpy.data.scenes['Scene'].rate21, bpy.data.scenes['Scene'].rate22, bpy.data.scenes['Scene'].rate23, bpy.data.scenes['Scene'].rate24, bpy.data.scenes['Scene'].rate25, bpy.data.scenes['Scene'].rate26, bpy.data.scenes['Scene'].rate27, bpy.data.scenes['Scene'].rate28, bpy.data.scenes['Scene'].rate29, bpy.data.scenes['Scene'].rate30, bpy.data.scenes['Scene'].rate31, bpy.data.scenes['Scene'].rate32, bpy.data.scenes['Scene'].rate33, bpy.data.scenes['Scene'].rate34, bpy.data.scenes['Scene'].rate35, bpy.data.scenes['Scene'].rate36]

    #########################################################################################################################################
    # First initialize the variables of the rate values with any value. The purpose is to be able to put them into the 'items_var' array and assign them each of values of the 'rate_scene'(which are rate values assigned by the user) array using the for-loop given below.
    ##################################################################################################################################
        rate1_val=1; rate2_val=1; rate3_val=1; rate4_val=1; rate5_val=1; rate6_val=1; rate7_val=1; rate8_val=1; rate9_val=1; rate10_val=1;  rate11_val=1; rate12_val=1; rate13_val=1; rate14_val=1; rate15_val=1; rate16_val=1; rate17_val=1; rate18_val=1; rate19_val=1; rate20_val=1; rate21_val=1; rate22_val=1; rate23_val=1; rate24_val=1; rate25_val=1; rate26_val=1; rate27_val=1; rate28_val=1; rate29_val=1; rate30_val=1; rate31_val=1; rate32_val=1; rate33_val=1; rate34_val=1; rate35_val=1; rate36_val=1
    # collect all the default initialized rate values in the 'rates_var' array
        rates_var=[rate1_val,rate2_val,rate3_val,rate4_val,rate5_val,rate6_val,rate7_val,rate8_val,rate9_val,rate10_val,rate11_val,rate12_val,rate13_val,rate14_val,rate15_val,rate16_val, rate17_val, rate18_val, rate19_val, rate20_val, rate21_val,rate22_val, rate23_val, rate24_val, rate25_val,rate26_val, rate27_val, rate28_val,rate29_val, rate30_val, rate31_val, rate32_val,rate33_val, rate34_val, rate35_val, rate36_val]
    #re-assign 'rates_var' members with rate values, the user assigned, as extracted from the 'rates_scene' array.
        for i in range(0, len(rates_scene)):
            rates_var[i]= rates_scene[i]
        par_values = []
    ##############################################################################################################################
    # Only rate values that range between 1 and 6 will be considered for breeding. Eventhough currently, no algorithm is included to make breeding #more degree specific between the six different rate values for breeding. In other words all values(b/n 1 and 6) are considered similar during breeding. Actions rated zero(0) are mutated.
    ###############################################################################################################################
        for items in range(0, len(rates_var)):
            if 1 <= rates_var[items] <= 6:
                par_values.append(rates_var[items])

        p1= 0; p2= 0; p3= 0; p4= 0; p5= 0; p6= 0; p7= 0; p8= 0; p9= 0; p10= 0; p11= 0; p12= 0; p13= 0; p14= 0; p15= 0; p16= 0; p17= 0; p18= 0; p19= 0; p20= 0; p21= 0; p22= 0; p23= 0; p24= 0; p25= 0; p26= 0; p27= 0; p28= 0; p29= 0; p30= 0; p31= 0; p32= 0; p33= 0; p34= 0; p35= 0; p36= 0


        action_flag = [p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12,p13, p14, p15, p16, p17, p18, p19, p20, p21, p22, p23, p24, p25, p26, p27, p28, p29, p30, p31, p32, p33, p34, p35, p36]

        action_str = ['action1', 'action2', 'action3', 'action4', 'action5', 'action6', 'action7', 'action8', 'action9', 'action10', 'action11', 'action12','action13', 'action14', 'action15', 'action16',\
               'action17', 'action18', 'action19', 'action20', 'action21', 'action22', 'action23', 'action24','action25', 'action26', 'action27', 'action28', 'action29', 'action30', 'action31', 'action32', 'action33', 'action34', 'action35', 'action36']

        rated_actions = []

    #count the members of the array; to know how often they repeat
    #for pm in range(0, len(par_mutate)):
        array_new = []

        for i in range(0, len(par_values)):
            check_once = 0
            for j in range(0, len(rates_var)):
                if par_values[i] == rates_var[j] and action_flag[j] == 0 and check_once == 0:
                    check_once = 1
                    action_flag[j] = 1
                    rated_actions.append(action_str[j])


        print('par_values')
        print(par_values)
        print('rates_var')
        print(rates_var)
        print('action_str')
        print(action_str)
        print('rated_actions')
        print(rated_actions)
    #Except the 'GST-all' action and actions with the word 'child' in their name all actions that contain 'GST-' in their
    #names will be collected into the 'expressio_list' array and used during the GA computation
        expression_list = []
        for i, action in enumerate(bpy.data.actions):
            #if action.name.startswith("GST-") and "child" not in action.name and "GST-all" not in action.name:
            for exp_name in popn:
                if action.name == exp_name:
                    expression_list.append(action)
        print (expression_list)

        expression_list = []
        for i, action in enumerate(bpy.data.actions):
            #if action.name.startswith("GST-") and "child" not in action.name and "GST-all" not in action.name:
            for exp_name in popn:
                if action.name == exp_name:
                    expression_list.append(action)
        print ("IGA expression_list")
        print (expression_list)


        head_fcurves = ['pose.bones["head"].location', 'pose.bones["head-target"].location', 'pose.bones["head"].rotation_euler', 'pose.bones["DEF-neck"].rotation_euler'] 
        mouth_fcurves = ['pose.bones["mouth_D_L"].location', 'pose.bones["mouth_D_R"].location', 'pose.bones["mouth_U_L"].location', 'pose.bones["mouth_U_R"].location', 'pose.bones["mouth_C_R"].location', 'pose.bones["mouth_C_L"].location', 'pose.bones["lip_C_L"].location', 'pose.bones["lip_D"].location', 'pose.bones["lip_D_L"].location', 'pose.bones["lip_D_R"].location', 'pose.bones["lip_U_L"].location', 'pose.bones["chin"].location', 'pose.bones["lip_U"].location', 'pose.bones["lip_U_R"].location']
        eyes_fcurves = ['pose.bones["brow.C"].location', 'pose.bones["brow.R"].location', 'pose.bones["brow.L"].location', 'pose.bones["brow.inner.R"].location', 'pose.bones["eye.offset"].location', 'pose.bones["brow.inner.L"].location']
        head_eyes_fcurves = ['pose.bones["head"].location', 'pose.bones["head-target"].location', 'pose.bones["head"].rotation_euler', 'pose.bones["brow.C"].location', 'pose.bones["brow.R"].location', 'pose.bones["brow.L"].location', 'pose.bones["brow.inner.R"].location', 'pose.bones["eye.offset"].location', 'pose.bones["brow.inner.L"].location']
        head_mouth_fcurves = ['pose.bones["head"].location', 'pose.bones["head-target"].location', 'pose.bones["head"].rotation_euler', 'pose.bones["mouth_D_L"].location', 'pose.bones["mouth_D_R"].location', 'pose.bones["mouth_U_L"].location', 'pose.bones["mouth_U_R"].location', 'pose.bones["mouth_C_R"].location', 'pose.bones["mouth_C_L"].location', 'pose.bones["lip_C_L"].location', 'pose.bones["lip_D"].location', 'pose.bones["lip_D_L"].location', 'pose.bones["lip_D_R"].location', 'pose.bones["lip_U_L"].location', 'pose.bones["chin"].location', 'pose.bones["lip_U"].location', 'pose.bones["lip_U_R"].location']
        mouth_eyes_fcurves = ['pose.bones["mouth_D_L"].location', 'pose.bones["mouth_D_R"].location', 'pose.bones["mouth_U_L"].location', 'pose.bones["mouth_U_R"].location', 'pose.bones["mouth_C_R"].location', 'pose.bones["mouth_C_L"].location', 'pose.bones["lip_C_L"].location', 'pose.bones["lip_D"].location', 'pose.bones["lip_D_L"].location', 'pose.bones["lip_D_R"].location', 'pose.bones["lip_U_L"].location', 'pose.bones["chin"].location', 'pose.bones["lip_U"].location', 'pose.bones["lip_U_R"].location','pose.bones["brow.C"].location', 'pose.bones["brow.R"].location', 'pose.bones["brow.L"].location', 'pose.bones["brow.inner.R"].location', 'pose.bones["eye.offset"].location', 'pose.bones["brow.inner.L"].location']

#Here save/categorize the set of actions to be bred/crossed-over and/or to be mutated in to the arrays 'expression_rated_array' and 'action_mutate' respectively.
        expression_rated_array = []
        action_mutate = []

        for i in range(0, len(expression_list)):
            exp = expression_list[i]
            if rates_var[i] > 0:
                expression_rated_array.append(exp)
            else:
                action_mutate.append(exp)
                
            

        if weight_mutation > 0:
            a = 0
            while a < len(action_mutate):
                for i in range(0, len(action_mutate[a].fcurves.items())):
                    random_num = random.randint(1,8)
                    control_points = len(action_mutate[a].fcurves[i].keyframe_points.items())
                    ii = 0
                    while ii < control_points:
                        if random_num == 2:
                            val = random.uniform(0, float(weight_mutation))
                            keyframe = action_mutate[a].fcurves[i].keyframe_points[ii].co[1] = val
                            mutated_keyframes = action_mutate[a].fcurves[i].keyframe_points[ii].co
                        #print ('mutated_keyframes')
                        #print (action_mutate[a].fcurves[i].group.name)
                        ii += 1
                    print(action_mutate[a])
                a += 1
                



        # use a reversed population as a starting population if the random generator chose the popn_range to be 2.(The second part)
        if popn_range == 2:
            expression_rated_array = list(reversed(expression_rated_array))

    #################################################################################################################
    # The block below copies a parent action (into 'copied_gesture'). So the first children generated at the children panel will be the parents
    # then the during subsequent computation the children will be copied into the children storage panel in the UI.
    #################################################################################################################
       
    # Only the parents that are included in the Cut-and-splice crossover will be copied to the children panel.
        expression_list_copy  = expression_rated_array
        popn_cutandsplice = popn_cutandsplice - len(action_mutate)
        copied_gesture = []
        for i in range(0, popn_cutandsplice):
            copied_gesture.append(expression_list_copy[i].copy())
       
         #just initialize a copied expression with a '0' appended to its name. This is useful just till the first generation (Since no expression are available with a generation number appended to their names before the 1st generation)
        if popn_cutandsplice > 1:
            copied_gesture[0].name = expression_list_copy[0].name[4:] + " " + str(0) 
        else:
             print("Low total number of expressions for the cut-and-splice crossover:")
             print(popn_cutandsplice)

    ### counts the the number of gestures with character 'child' in their name. This count helps in determining the generation number of a child gesture ###

        max_gen = 1
        exp_gen = ''
        for child_gesture in bpy.data.actions:
            if "IGA-child" in child_gesture.name:
                if len(child_gesture.name.split(" ")) > 1:
                    exp_gen = float(child_gesture.name.split(" ")[1]) # to handle string numbers that contain '.'
                    exp_gen = int(exp_gen)
                    if max_gen < exp_gen:
                        max_gen = exp_gen
            
        for j in range(0, len(copied_gesture)):
            copied_gesture[j].name = "IGA-child-" + expression_list_copy[j].name[4:] + " " +  str(int(max_gen+1))

######################################################################################################################




        fcurves_to_be_removed = []
        fcurves_to_be_inserted = []

    # Single group of fcurves (head, eyes or mouth) removal from the first parent 'cut and splice' breeding technique style
        cut_and_splice_breeding_style = random.randint(1,6)
        print("cut and splice breeding style:")
        print(cut_and_splice_breeding_style)
        if cut_and_splice_breeding_style == 1:
            fcurves_to_be_removed = head_fcurves
        elif cut_and_splice_breeding_style == 2:
            fcurves_to_be_removed = eyes_fcurves
        elif cut_and_splice_breeding_style == 3:
            fcurves_to_be_removed = mouth_fcurves
    #Two group of fcurves(such as head and eyes, head and mouth, eyes and mouth etc.) removal from the first parent 'cut and splice' breeding technique style
        elif cut_and_splice_breeding_style == 4:
            fcurves_to_be_removed = head_eyes_fcurves

        elif cut_and_splice_breeding_style == 5:
            fcurves_to_be_removed = head_mouth_fcurves

        elif cut_and_splice_breeding_style == 6:
            fcurves_to_be_removed = mouth_eyes_fcurves

        #cross over function
        a = 0
        action_counter = -1
        
        #while a < len(expression_rated_array)-1:
        while a < popn_cutandsplice-1:
            action_counter += 1
            print ("cut and splice expressions")
            print (expression_rated_array[a])
            print (expression_rated_array[a+1])

    # remember here, generally we can insert the capability to integrate which part of face's part is selected in the checkbox
            #print('len(child_action.fcurves.items()_before removal)')
            #print(expression_rated_array[a].name)
            #print(len(expression_rated_array[a].fcurves.items()))
            #print("len of curves_to_be_removed")
            #print(fcurves_to_be_removed)


    #############################################################
    ###Algorithm for the cut and splice breeding technique###
    ##############################################################


    # since a child_action which is copy/replica of the parent 1 is created already,
    # Then it removes the head part related fcurves from the child_action but leaves eyes and mouth related fcurves as they are
    # And it copies the head related fcurves from parent 2 to child_action



    # Subtract '2' from the total curves of the the reason is the start of the fcurves index is -1 for the action (bpy.data.actions['name'].fcurves[-1].data_path)
            max_fcurve_size = len(expression_rated_array[a].fcurves.items())-1 

            while max_fcurve_size >= 0:
                fcur = 0
                flag = 0 

                while fcur < len(fcurves_to_be_removed) and flag == 0:
                    if expression_rated_array[a].fcurves[max_fcurve_size].data_path == fcurves_to_be_removed[fcur]:
                        expression_rated_array[a].fcurves.remove(expression_rated_array[a].fcurves[max_fcurve_size])
                        flag = 1
                        print("removed")
                    fcur += 1
                max_fcurve_size -= 1
        
            print('len(child_action.fcurves.items()_after removal)')
            print(expression_rated_array[a].name)
            print(len(expression_rated_array[a].fcurves.items()))
    

    #copy head related fcurves from the 2nd parent to the child

        # selectedObj.animation_data.action = bpy.data.actions.new(name="GST-random1")
        #curve = selectedObj.animation_data.action.fcurves
            curve = expression_rated_array[a].fcurves

    #just for naming convienance copy the values of the fcurves_to_be_removed to a new variable
            fcurves_to_be_inserted = fcurves_to_be_removed

    #Making the array of fcurves an outer loop help us in detecting an array of fcurves in the 2nd parent that can have the same data_path name
            for fcurves in range(0, len(fcurves_to_be_inserted)):
                duplicate_curve_name = 0
                for exp_fcurves in range(0, len(expression_rated_array[a+1].fcurves.items())):
                    if expression_rated_array[a+1].fcurves[exp_fcurves].data_path == fcurves_to_be_inserted[fcurves] and duplicate_curve_name == 0:
                        exp_fcurve_datapath = expression_rated_array[a+1].fcurves[exp_fcurves].data_path
                        exp_fcurve_index = expression_rated_array[a+1].fcurves[exp_fcurves].array_index
                      # Some fcurves are not assinged a group name, leaving it out is better, it generates errors during cross-over computation
                            #exp_fcurve_groupname = expression_rated_array[a+1].fcurves[exp_fcurves].group.name

              #creates a curve that have similar parameters to that of 2nd parent's chosen group of fcurves(such as the head part or mouth etc.) 
                        created_fcurve = curve.new(data_path = exp_fcurve_datapath, index = exp_fcurve_index)#, action_group = exp_fcurve_groupname)

              # counts the number of keyframe points in the specific head related fcurve of the 2nd parent
                        keyframe_count = len(expression_rated_array[a+1].fcurves[exp_fcurves].keyframe_points.items())

             # adds keyframe points, sized in number, equal to the retrieved count of keyframe points of the fcurve in the 2nd parent
                        created_fcurve.keyframe_points.add(keyframe_count)
                        duplicate_curve_name = 1

                        i = 0
                        while i < keyframe_count:
                            keypoint_co = expression_rated_array[a+1].fcurves[exp_fcurves].keyframe_points[i].co[0]
                            keyval_co = expression_rated_array[a+1].fcurves[exp_fcurves].keyframe_points[i].co[1]
                            created_fcurve.keyframe_points[i].co = keypoint_co, keyval_co
                            #print("copied_fcurve_keyframe_points")
                            #print(created_fcurve.keyframe_points[i].co)
                            i=i+1
            a += 1
     
        # Reset the list of selected parts of expressions after a generation             
        for s in range(0, len(self.selected_actions)):
            for sel in range(0, len(self.selected_actions)): 
                self.selected_actions.remove(self.selected_actions[sel])
                break



    #*****************************************************************************************************************************

    def childGenerator_blendIGA(self, popn, popn_blend, popn_range, weight_mutation, selected_group):
        print("blend population:")
        print(popn_blend)

    
    # An array that holds list of the rate values as inserted by the user. Default rate value set to 1 as defined in function GA_register().
        rates_scene = [bpy.data.scenes['Scene'].rate1, bpy.data.scenes['Scene'].rate2, bpy.data.scenes['Scene'].rate3, bpy.data.scenes['Scene'].rate4, bpy.data.scenes['Scene'].rate5, bpy.data.scenes['Scene'].rate6, bpy.data.scenes['Scene'].rate7, bpy.data.scenes['Scene'].rate8, bpy.data.scenes['Scene'].rate9, bpy.data.scenes['Scene'].rate10, bpy.data.scenes['Scene'].rate11, bpy.data.scenes['Scene'].rate12, bpy.data.scenes['Scene'].rate13, bpy.data.scenes['Scene'].rate14, bpy.data.scenes['Scene'].rate15, bpy.data.scenes['Scene'].rate16, bpy.data.scenes['Scene'].rate17, bpy.data.scenes['Scene'].rate18, bpy.data.scenes['Scene'].rate19, bpy.data.scenes['Scene'].rate20, bpy.data.scenes['Scene'].rate21, bpy.data.scenes['Scene'].rate22, bpy.data.scenes['Scene'].rate23, bpy.data.scenes['Scene'].rate24, bpy.data.scenes['Scene'].rate25, bpy.data.scenes['Scene'].rate26, bpy.data.scenes['Scene'].rate27, bpy.data.scenes['Scene'].rate28, bpy.data.scenes['Scene'].rate29, bpy.data.scenes['Scene'].rate30, bpy.data.scenes['Scene'].rate31, bpy.data.scenes['Scene'].rate32, bpy.data.scenes['Scene'].rate33, bpy.data.scenes['Scene'].rate34, bpy.data.scenes['Scene'].rate35, bpy.data.scenes['Scene'].rate36]


    #Put the rates_scene array (list of values put by users into the rate values textboxes; default set to 1) into a dictionary 'rates_var'.
        rates_var =[]
        for i in range(0, len(rates_scene)):
            rates_var.append(rates_scene[i])


######################################################################################
#Define a function here to insert/load default list of expressions to be imported if the user doesn't want to import.
######################################################################################


#Expressions imported into the IGA panel will be copied from 'popn' list to 'expression_list'.
        expression_list = []
        for i, action in enumerate(bpy.data.actions):
        #if action.name.startswith("GST-") and "child" not in action.name and "GST-all" not in action.name:
            for exp_name in popn:
                if action.name == exp_name:
                    expression_list.append(action)
        print ("IGA expression_list")
        print (expression_list)
     



#Here save/categorize the set of actions to be bred/crossed-over and/or to be mutated in to the arrays 'expression_rated_array' and 'action_mutate' respectively.
        expression_rated_array = []
        action_mutate = []

        for i in range(0, len(expression_list)):
            exp = expression_list[i]
            if rates_var[i] > 0:
                expression_rated_array.append(exp)
            else:
                action_mutate.append(exp)

        print ('List of actions to be cross-overed')
        print (expression_rated_array)

        print('List of actions to be mutated')
        print(action_mutate)



##################################################################
                 ### Mutation Algorithm ###
##################################################################

#################################################################################################################################################
#mutation function; we mutate actions that are rated lowest(with value 0) by the user.
# In genetic algorithm the mutation rate is adviced to be very small. Heeding to this idea,here, mutation level
# of an action which is provided for mutation(an action rated 0 by the user) is kept to as low as in average to 1/8 of the total fcurves of
# the give action; assuming that random number generator has equal distribuion.
# Besides to reduce an exaggerated effect of mutation on the selected fcurves, their keyframes' mutation are limited to random values b/n 0 and 0.02. If the mutated actions tend to be very small then you can increase mutation by narrowing the random number range in 'random_num' and also you can start increasing random range of 'val' and see the effects.
#################################################################################################################################################

        if weight_mutation > 0:
            a = 0
            while a < len(action_mutate):
                for i in range(0, len(action_mutate[a].fcurves.items())):
                    random_num = random.randint(1,8)
                    control_points = len(action_mutate[a].fcurves[i].keyframe_points.items())
                    ii = 0
                    while ii < control_points:
                        if random_num == 2:
                            val = random.uniform(0, float(weight_mutation))
                            keyframe = action_mutate[a].fcurves[i].keyframe_points[ii].co[1] = val
                            mutated_keyframes = action_mutate[a].fcurves[i].keyframe_points[ii].co
                        #print ('mutated_keyframes')
                        #print (action_mutate[a].fcurves[i].group.name)
                        ii += 1
                    print(action_mutate[a])
                a += 1

        ##########################################################
        ### Algorithm to Keep Interesting Sub-Parts (Head, Eyes and Mouth) from the Parent Facial Expressions to the Next Generation ###
        ##########################################################
    
        #selected_group = EvaAPI().select_Facialparts(EvaAPI)
        print("selected_group")
        print(selected_group)

        # Neck is included within the 'head_fcurves but since changes in values seem very sensative, the values have to be limited within limited ranges'
        head_fcurves = ['pose.bones["head"].location', 'pose.bones["head-target"].location', 'pose.bones["head"].rotation_euler', 'pose.bones["DEF-neck"].rotation_euler']
        mouth_fcurves = ['pose.bones["mouth_D_L"].location', 'pose.bones["mouth_D_R"].location', 'pose.bones["mouth_U_L"].location', 'pose.bones["mouth_U_R"].location', 'pose.bones["mouth_C_R"].location', 'pose.bones["mouth_C_L"].location', 'pose.bones["lip_C_L"].location', 'pose.bones["lip_D"].location', 'pose.bones["lip_D_L"].location', 'pose.bones["lip_D_R"].location', 'pose.bones["lip_U_L"].location', 'pose.bones["chin"].location', 'pose.bones["lip_U"].location', 'pose.bones["lip_U_R"].location']
        eyes_fcurves = ['pose.bones["blink_rate"].location', 'pose.bones["eyelid_LO_R"].location', 'pose.bones["eyelid_LO_L"].location', 'pose.bones["eyelid_blink_UP_R"].location', 'pose.bones["eyelid_U_R"].location', 'pose.bones["eyelid_U_L"].location', 'pose.bones["eye_L"].location', 'pose.bones["eye_R"].location']




        ###########################################################
        ### the very start of crossover function ###
        ###########################################################
    
        # use a reversed population as a starting population if the random generator chose the popn_range to be 2.(The second part)
        if popn_range == 2:
            expression_rated_array = list(reversed(expression_rated_array))


##############################################################################################################################################
# The block below copies a parent action (into 'copied_gesture'). So the first children generated at the children panel will be the parents
# then the during subsequent computation the children will be copied into the children storage panel in the UI.
##############################################################################################################################################
        
        # Only the parents that are included in the blend breeding will be copied to the child panel.
        expression_list_copy  = expression_rated_array
        popn_blend = popn_blend - len(action_mutate)
        copied_gesture = []
        for i in range(0, popn_blend):
            copied_gesture.append(expression_list_copy[i].copy())
       
        #just initialize a copied expression with a '0' appended to its name. This is useful just till the first generation (Since no expression are available with a generation number appended to their names before the 1st generation)
        if popn_blend>1:
            copied_gesture[0].name = expression_list_copy[0].name + " " + str(0)
        else:
            print("Low total number of expressions for the blend crossover:")
            print(popn_blend)

    ### counts the the number of gestures with character 'child' in their name. This count helps in determining the generation number of a child gesture ###
        max_gen = 1
        exp_gen = ''
        for child_gesture in bpy.data.actions:
            if "IGA-child" in child_gesture.name:
                if len(child_gesture.name.split(" ")) > 1:
                    exp_gen = float(child_gesture.name.split(" ")[1]) # to handle string numbers that contain '.'
                    exp_gen = int(exp_gen)
                    if max_gen < exp_gen:
                        max_gen = exp_gen
            
        for j in range(0, len(copied_gesture)):
            copied_gesture[j].name = "IGA-child-" + expression_list_copy[j].name[4:] + " " +  str(int(max_gen+1))
######################################################################################################################




        a = 0
        action_counter = -1
        #while a < len(expression_rated_array)-1 and action_counter < 10:
        while a < popn_blend-1:
            action_counter += 1
        #print ('expression')
        #print (len(array_actions_rated))
            print ("blend expressions")
            print (expression_rated_array[a])
            print (expression_rated_array[a+1])
            f = 0
            c = 2



        #if action_counter < 33:# determines the maximum number of children that can be generated maximum 34 (since action_counter begins at -1) children per generation
        #gp = expression_rated_array[a].fcurves[i].group.name
        #for i in range(0, len(expression_rated_array[a].fcurves.items())):

        ##########################################################################################################################################################
        # 'jump_one_curve' is a variable that indicates the starting value of fcurve of an action to be skipped during the cross-over algorithm.
        # It's randomly assigned either 0 or 1. This random initialization can create two variations of children of the same set of parents
        # bred during different times. The reason is the fcurve to be jumped(based on the principle of uniform-based genetic algorithm) can be either the even one
        # fcurves of the first parent(assuming fcurves satisfy the data_path and array_index similarity condition between the two parents) if jump_one_curve is '2'
        # or the odd numbered fcurves of the first parent if jump_one_curve.
        ##########################################################################################################################################################

            jump_one_curve = random.randint(2,3)

            count_ticked_per_emotion = []
            ticked_fcurves = []
            for selected in range(0, len(selected_group)):
                selected_gp = selected_group[selected].split("_")
                if expression_rated_array[a].name == selected_gp[0]: #selected_gp[0] is the first part(the name) of the facial expression
                    count_ticked_per_emotion.append(selected_gp[1]) # store the second part(such as head, eyes or mouth) from the expression


# Here compute the length of count_ticked_per_emotion(values can range from 0 (when non is ticked) to 3 (when all three are ticked))

       # print("count_ticked_per_facial expression")
       #print(count_ticked_per_emotion)

            if len(count_ticked_per_emotion) >= 1:   
             #multi checkbox selection per emotion
                for tick_no in range(0, len(count_ticked_per_emotion)):
                    if count_ticked_per_emotion[tick_no] == 'eyes':
                        for eyes in range(0, len(eyes_fcurves)):
                            ticked_fcurves.append(eyes_fcurves[eyes])
                    if count_ticked_per_emotion[tick_no] == 'mouth':
                        for mouth in range(0, len(mouth_fcurves)):
                            ticked_fcurves.append(mouth_fcurves[mouth])
                    if count_ticked_per_emotion[tick_no] == 'head':
                        for head in range(0, len(head_fcurves)):
                            ticked_fcurves.append(head_fcurves[head])



  # Group of fcurves that are ticked are supposed to be kept during the next children generation so the first block of for-loop below jumps/skips all the ticked fcurves during breeding
            for i in range(0, len(expression_rated_array[a].fcurves.items())):
                checkbox_ticked = 0
        # ticked_fcurves contains all the fcurves within the ticked checkboxes
                for ticked in range(0, len(ticked_fcurves)):
                    if expression_rated_array[a].fcurves[i].data_path == ticked_fcurves[ticked]:
               # print ("iterate next")
               # print (ticked_fcurves[ticked])
               # print ("expression_rated_array[a].fcurves[i].data_path")
               # print (expression_rated_array[a].fcurves[i].data_path)
                        checkbox_ticked = 1

                if checkbox_ticked == 0:
                    for j in range(0, len(expression_rated_array[a+1].fcurves.items())):
                        datapath_1 = expression_rated_array[a].fcurves[i].data_path
                        arrayindex_1 = expression_rated_array[a].fcurves[i].array_index

                        datapath_2 = expression_rated_array[a+1].fcurves[j].data_path
                        arrayindex_2 = expression_rated_array[a+1].fcurves[j].array_index

# A condition that checks if given fcurves of both parents match and if 'jump_one_curve' is even it replaces the fcurves of the first parent
# is replaced by similar fcurve from the second parent, to constitute the child action. The initial value of the 'jump_one_curve' is randomly
# initialized above either to 2 or 3.

#
 # As an experiment idea, the 'jump_one_curve' can be made to be generated randomly continuously which would drive the chidren generation to #be very much random.

                        if datapath_1 == datapath_2 and arrayindex_1 == arrayindex_2 and jump_one_curve % 2 == 0:

                            jump_one_curve += 1

                   ##############################################################################################################################################
 ## Generating the 'jump_one_curve' randomly can give result to strange children actions as a result of the randomized jumping/the way to
 ## jump between fcurves either to be replaced or skipped. The current style is once it starts replacing the odd fcurves of the first parent
 ## by the second parent it continues doing so for the full set of fcurves between the two parents which constitute the child action.
 ## So you can imagine if the way we replace/skip fcurves b/n two parents in order to build a child is very random. Of course the result can be
 ## a strange action or a can also be very one-sided (a child action inclined in similarity to one of the parents). But if you think it is
 ## worth experimenting then comment the above 'jump_one_curve' and uncomment the 'jump_one_curve' statement below.
                   ##############################################################################################################################################

                   # jump_one_curve = random.randint(2,3)

                            c = control_points = len(expression_rated_array[a].fcurves[i].keyframe_points.items())-1
                            b = control_points_2 = len(expression_rated_array[a+1].fcurves[j].keyframe_points.items())-1


                    # Checks the condition that the keyframes,are to be bred, of the given fcurve in the first parent are more in number
                    # than the keyframes to of the same fcurve in the second parents, and then removes the excess keyframes in the first
                    # parent
                            if control_points - control_points_2 > 0:
                                while c > b:
                                    expression_rated_array[a].fcurves[i].keyframe_points.remove(expression_rated_array[a].fcurves[i].keyframe_points[c])
                                    c -= 1
                   # Check the condition if the first parent's keyframes of the given fcurve are less than the second parent. If such is the
                   # case then add keyframe to the first parent to match the number keyframes of the given fcurve of the second parent.
                            elif control_points - control_points_2 < 0:
                                cont_val =  control_points_2 - control_points
                                expression_rated_array[a].fcurves[i].keyframe_points.add(cont_val)

                            control_points_new = len(expression_rated_array[a].fcurves[i].keyframe_points.items())
                    #control_points_2 = len(expression_rated_array[a+1].fcurves[j].keyframe_points.items())
                    
                       				    ### Blend Operator###
                    # Sum and get average of sum of corresponding keyframe points of similar fcurves found in both expressions and then put it on the first expression 
                            ii = 0
                            while ii < control_points_new:
                                for x in range(0, len(head_fcurves)):
                                    if expression_rated_array[a].fcurves[i].data_path == head_fcurves[x]:

                                        # values and keyframes of expression1
                                        val1 = expression_rated_array[a+1].fcurves[j].keyframe_points[ii].co[1]
                                        keyframe1 = expression_rated_array[a+1].fcurves[j].keyframe_points[ii].co[0]

                                        # values and keyframes of expression2
                                        val2 = expression_rated_array[a+1].fcurves[j].keyframe_points[ii].co[1]
                                        keyframe2 = expression_rated_array[a+1].fcurves[j].keyframe_points[ii].co[0]
                                        
                                       # sum and get average of the keyframe and value parameters
                                        average_val = (val1 + val2)/2 
                                        average_keyframe = (keyframe1 + keyframe2)/2
                                        expression_rated_array[a].fcurves[i].keyframe_points[ii].co = average_keyframe, average_val
                                        ee = expression_rated_array[a].fcurves[i].keyframe_points[ii].co
                                   # print(expression_rated_array[a].fcurves[i].data_path)
                                   # print(ee)
                                   # print(expression_rated_array[a+1].fcurves[j].data_path)
                                   # print(expression_rated_array[a+1].fcurves[j].keyframe_points[ii].co)
                                   # print ("true")
                                ii += 1
                # An elif condition to increment 'jump_one_curve' when replacing of fcurve from the second parent is skipped.
                        elif datapath_1 == datapath_2 and arrayindex_1 == arrayindex_2 and jump_one_curve % 2 == 1:
                            jump_one_curve += 1

                 ### for randomized generation of 'jump_one_curve'###
                  # jump_one_curve = random.randint(2,3)

            #i += 1
            a += 1 
        print("action_counter")
        print(action_counter)
     
        # Reset the list of selected parts of expressions after a generation             
        for s in range(0, len(self.selected_actions)):
            for sel in range(0, len(self.selected_actions)): 
                self.selected_actions.remove(self.selected_actions[sel])
                break



    # This function is to be called by the blenderplayback.py (registration of GA related UI components)
    def GA_register():

        bpy.types.Scene.rate1 = bpy.props.IntProperty(name = "rate1");bpy.context.scene['rate1'] = 1;
        bpy.types.Scene.rate2 = bpy.props.IntProperty(name = "rate2");bpy.context.scene['rate2'] = 1;
        bpy.types.Scene.rate3 = bpy.props.IntProperty(name = "rate3");bpy.context.scene['rate3'] = 1;
        bpy.types.Scene.rate4 = bpy.props.IntProperty(name = "rate4");bpy.context.scene['rate4'] = 1;
        bpy.types.Scene.rate5 = bpy.props.IntProperty(name = "rate5");bpy.context.scene['rate5'] = 1;
        bpy.types.Scene.rate6 = bpy.props.IntProperty(name = "rate6");bpy.context.scene['rate6'] = 1;
        bpy.types.Scene.rate7 = bpy.props.IntProperty(name = "rate7");bpy.context.scene['rate7'] = 1;
        bpy.types.Scene.rate8 = bpy.props.IntProperty(name = "rate8");bpy.context.scene['rate8'] = 1;
        bpy.types.Scene.rate9 = bpy.props.IntProperty(name = "rate9");bpy.context.scene['rate9'] = 1;
        bpy.types.Scene.rate10 = bpy.props.IntProperty(name = "rate10");bpy.context.scene['rate10'] = 1;
        bpy.types.Scene.rate11 = bpy.props.IntProperty(name = "rate11");bpy.context.scene['rate11'] = 1;
        bpy.types.Scene.rate12 = bpy.props.IntProperty(name = "rate12");bpy.context.scene['rate12'] = 1;
        bpy.types.Scene.rate13 = bpy.props.IntProperty(name = "rate13");bpy.context.scene['rate13'] = 1;
        bpy.types.Scene.rate14 = bpy.props.IntProperty(name = "rate14");bpy.context.scene['rate14'] = 1;
        bpy.types.Scene.rate15 = bpy.props.IntProperty(name = "rate15");bpy.context.scene['rate15'] = 1;
        bpy.types.Scene.rate16 = bpy.props.IntProperty(name = "rate16");bpy.context.scene['rate16'] = 1;
        bpy.types.Scene.rate17 = bpy.props.IntProperty(name = "rate17");bpy.context.scene['rate17'] = 1;
        bpy.types.Scene.rate18 = bpy.props.IntProperty(name = "rate18");bpy.context.scene['rate18'] = 1;
        bpy.types.Scene.rate19 = bpy.props.IntProperty(name = "rate19");bpy.context.scene['rate19'] = 1;
        bpy.types.Scene.rate20 = bpy.props.IntProperty(name = "rate20");bpy.context.scene['rate20'] = 1;
        bpy.types.Scene.rate21 = bpy.props.IntProperty(name = "rate21");bpy.context.scene['rate21'] = 1;
        bpy.types.Scene.rate22 = bpy.props.IntProperty(name = "rate22");bpy.context.scene['rate22'] = 1;
        bpy.types.Scene.rate23 = bpy.props.IntProperty(name = "rate23");bpy.context.scene['rate23'] = 1;
        bpy.types.Scene.rate24 = bpy.props.IntProperty(name = "rate24");bpy.context.scene['rate24'] = 1;
        bpy.types.Scene.rate25 = bpy.props.IntProperty(name = "rate25");bpy.context.scene['rate25'] = 1;
        bpy.types.Scene.rate26 = bpy.props.IntProperty(name = "rate26");bpy.context.scene['rate26'] = 1;
        bpy.types.Scene.rate27 = bpy.props.IntProperty(name = "rate27");bpy.context.scene['rate27'] = 1;
        bpy.types.Scene.rate28 = bpy.props.IntProperty(name = "rate28");bpy.context.scene['rate28'] = 1;
        bpy.types.Scene.rate29 = bpy.props.IntProperty(name = "rate29");bpy.context.scene['rate29'] = 1;
        bpy.types.Scene.rate30 = bpy.props.IntProperty(name = "rate30");bpy.context.scene['rate30'] = 1;
        bpy.types.Scene.rate31 = bpy.props.IntProperty(name = "rate31");bpy.context.scene['rate31'] = 1;
        bpy.types.Scene.rate32 = bpy.props.IntProperty(name = "rate32");bpy.context.scene['rate32'] = 1;
        bpy.types.Scene.rate33 = bpy.props.IntProperty(name = "rate33");bpy.context.scene['rate33'] = 1;
        bpy.types.Scene.rate34 = bpy.props.IntProperty(name = "rate34");bpy.context.scene['rate34'] = 1;
        bpy.types.Scene.rate35 = bpy.props.IntProperty(name = "rate35");bpy.context.scene['rate35'] = 1;
        bpy.types.Scene.rate36 = bpy.props.IntProperty(name = "rate36");bpy.context.scene['rate36'] = 1;
        bpy.types.Scene.rate25 = bpy.props.IntProperty(name = "rate37");bpy.context.scene['rate37'] = 1;
        bpy.types.Scene.rate26 = bpy.props.IntProperty(name = "rate38");bpy.context.scene['rate38'] = 1;
        bpy.types.Scene.rate27 = bpy.props.IntProperty(name = "rate39");bpy.context.scene['rate39'] = 1;
        bpy.types.Scene.rate28 = bpy.props.IntProperty(name = "rate40");bpy.context.scene['rate40'] = 1;
        bpy.types.Scene.rate29 = bpy.props.IntProperty(name = "rate41");bpy.context.scene['rate41'] = 1;
        bpy.types.Scene.rate30 = bpy.props.IntProperty(name = "rate42");bpy.context.scene['rate42'] = 1;
        bpy.types.Scene.rate31 = bpy.props.IntProperty(name = "rate43");bpy.context.scene['rate43'] = 1;
        bpy.types.Scene.rate32 = bpy.props.IntProperty(name = "rate44");bpy.context.scene['rate44'] = 1;
        bpy.types.Scene.rate33 = bpy.props.IntProperty(name = "rate45");bpy.context.scene['rate45'] = 1;
        bpy.types.Scene.rate34 = bpy.props.IntProperty(name = "rate46");bpy.context.scene['rate46'] = 1;
        bpy.types.Scene.rate35 = bpy.props.IntProperty(name = "rate47");bpy.context.scene['rate47'] = 1;
        bpy.types.Scene.rate36 = bpy.props.IntProperty(name = "rate48");bpy.context.scene['rate48'] = 1;

        bpy.types.Scene.action_currentname = bpy.props.StringProperty(name = "action_currentname")
        bpy.context.scene['action_currentname'] = "Current name"
  
        bpy.types.Scene.action_newname = bpy.props.StringProperty(name = "action_newname")
        bpy.context.scene['action_newname'] = "New name"
   
    # UI definition for adding and removing actions to the GA related panel list.
        bpy.types.Scene.action_add_IGA = bpy.props.StringProperty(name = "action_add_IGA")
        bpy.context.scene['action_add_IGA'] = "GST-amused : GST-nod-1 : GST-nod-2 : GST-nod-3 : GST-shake-2 : GST-shake-3 : GST-shake-4 : GST-think-UP : GST-thoughtful : GST-yawn-1"

    # UI definition for saving IGA based evolved actions into (of course, would work for other unsaved actions too).
        bpy.types.Scene.action_save_IGA = bpy.props.StringProperty(name = "action_save_IGA")
        bpy.context.scene['action_save_IGA'] = "IGA-amused : IGA-thoughtful : IGA-yawn-1"

    # UI definition for adding and removing actions to the GA related panel list.
        bpy.types.Scene.action_add = bpy.props.StringProperty(name = "action_add")
        bpy.context.scene['action_add'] = "Action to be Added"
  
        bpy.types.Scene.action_remove = bpy.props.StringProperty(name = "action_remove")
        bpy.context.scene['action_remove'] = "Permanent action remove"

        bpy.types.Scene.weight_uniform = bpy.props.FloatProperty(name = "weight_uniform");bpy.context.scene['weight_uniform'] = 0.8;
        bpy.types.Scene.weight_cutandsplice = bpy.props.FloatProperty(name = "weight_cutandsplice");bpy.context.scene['weight_cutandsplice'] = 0.2;
        bpy.types.Scene.weight_blend = bpy.props.FloatProperty(name = "weight_blend");bpy.context.scene['weight_blend'] = 0.0;
        bpy.types.Scene.weight_mutation = bpy.props.FloatProperty(name = "weight_mutation");bpy.context.scene['weight_mutation'] = 0.01;



    def GA_performgesture():
        '''Perform a new gesture.'''
        fail = False
        try:
            actionDatablock = bpy.data.actions[name]
        except KeyError:
            fail = True

        if fail:
            raise TypeError('Gesture \"' + name + '\" is not known')
            return

        # Check value for sanity
        checkValue(repeat, 1, 1000)
        checkValue(speed, 0.1, 10)
        checkValue(magnitude, 0, 1)
        checkValue(priority, 0, 1)

        # Create NLA track
        newTrack = self.deformObj.animation_data.nla_tracks.new()
        newTrack.name = name

        # Create strip
        newStrip = newTrack.strips.new(name=name, start=1, action=actionDatablock)
        duration = (newStrip.frame_end - newStrip.frame_start)
        newStrip.blend_type = 'ADD'
        newStrip.use_animated_time = True
        # Create the strip time function
        f = newStrip.fcurves.items()[0][1]
        # Point at 1st frame
        strip_time_kfp = f.keyframe_points.insert(1,0,{'FAST'})
        # force blink to play at 1.0 intensity
        if 'blink' in name.lower():
            magnitude = 1

        if magnitude < 1:
            newStrip.use_animated_influence = True
            newStrip.influence = magnitude

        # Create object and add to list
        gg = Gesture(name, newTrack, newStrip, duration=duration, speed=speed, \
         magnitude=magnitude, priority=priority, repeat=repeat, strip_time_kfp=strip_time_kfp)

        self.gesturesList_GA.append(gg)
        

    # GA related
    def GA_deletegesture():
        ''' internal use only, stops and deletes a gesture'''
        # remove from list
        #bpy.evaAnimationManager.gesturesList_GA.remove(bpy.evaAnimationManager.gesturesList_GA[0])
         
        self.gesturesList_GA.remove(gesture)

        # remove from Blender
        self.deformObj.animation_data.nla_tracks.remove(gesture.trackRef)
    



    #if __name__=='__main__':
    #    newGesture(self, name, repeat = 1, speed=1, magnitude=0.5, priority=1)



