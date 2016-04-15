import bpy

modes = ['keep_alive', 'listening', 'chatting']
active_mode = modes[2]

if active_mode == 'keep_alive':
    bpy.context.scene.actuators.ACT_blink_randomly.PARAM_interval_mean = 2.25
    bpy.context.scene.actuators.ACT_blink_randomly.PARAM_interval_variation = 0.7
    bpy.context.scene.actuators.ACT_blink_randomly.PARAM_blink_duration = 0.27
    
    bpy.context.scene.actuators.ACT_saccade.PARAM_interval_mean = 2.45
    bpy.context.scene.actuators.ACT_saccade.PARAM_interval_variation = 0.9
    bpy.context.scene.actuators.ACT_saccade.PARAM_paint_scale = 4
    bpy.context.scene.actuators.ACT_saccade.PARAM_eye_size = 15
    bpy.context.scene.actuators.ACT_saccade.PARAM_eye_distance = 100
    bpy.context.scene.actuators.ACT_saccade.PARAM_mouth_width = 90
    bpy.context.scene.actuators.ACT_saccade.PARAM_mouth_height = 27
    bpy.context.scene.actuators.ACT_saccade.PARAM_weight_eyes = 0.4
elif active_mode == 'listening':
    bpy.context.scene.actuators.ACT_blink_randomly.PARAM_interval_mean = 1.6
    bpy.context.scene.actuators.ACT_blink_randomly.PARAM_interval_variation = 0.8
    bpy.context.scene.actuators.ACT_blink_randomly.PARAM_blink_duration = 0.2
    
    bpy.context.scene.actuators.ACT_saccade.PARAM_interval_mean = 2.2
    bpy.context.scene.actuators.ACT_saccade.PARAM_interval_variation = 0.6
    bpy.context.scene.actuators.ACT_saccade.PARAM_paint_scale = 1
    bpy.context.scene.actuators.ACT_saccade.PARAM_eye_size = 11
    bpy.context.scene.actuators.ACT_saccade.PARAM_eye_distance = 80
    bpy.context.scene.actuators.ACT_saccade.PARAM_mouth_width = 50
    bpy.context.scene.actuators.ACT_saccade.PARAM_mouth_height = 13
    bpy.context.scene.actuators.ACT_saccade.PARAM_weight_eyes = 0.5
elif active_mode == 'chatting':
    bpy.context.scene.actuators.ACT_blink_randomly.PARAM_interval_mean = 1.6
    bpy.context.scene.actuators.ACT_blink_randomly.PARAM_interval_variation = 0.8
    bpy.context.scene.actuators.ACT_blink_randomly.PARAM_blink_duration = 0.2
   
    bpy.context.scene.actuators.ACT_saccade.PARAM_interval_mean = .8
    bpy.context.scene.actuators.ACT_saccade.PARAM_interval_variation = 0.9
    bpy.context.scene.actuators.ACT_saccade.PARAM_paint_scale = 3
    bpy.context.scene.actuators.ACT_saccade.PARAM_eye_size = 11.5
    bpy.context.scene.actuators.ACT_saccade.PARAM_eye_distance = 100
    bpy.context.scene.actuators.ACT_saccade.PARAM_mouth_width = 90
    bpy.context.scene.actuators.ACT_saccade.PARAM_mouth_height = 5
    bpy.context.scene.actuators.ACT_saccade.PARAM_weight_eyes = 0.4


