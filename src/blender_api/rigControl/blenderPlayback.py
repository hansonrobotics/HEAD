# This module sets up a modal operator in Blender to act
# as the animation playback service, and hosts other supporting test operators

# Frame rate for animation playback
framerateHz = 48

# System timer
defaultTimerHz = 48

import bpy
from .helpers import *

import pprint, time
import logging

logger = logging.getLogger('hr.blender_api.rigcontrol.blenderplayback')

class BLGlobalTimer(bpy.types.Operator):
    """Timer  Control"""
    bl_label = "Global Timer"
    bl_idname = 'wm.global_timer'
    # New property for global timer
    bpy.types.Scene.globalTimerStarted = bpy.props.BoolProperty(name = "globalTimerStarted", default=False)
    bpy.context.scene['globalTimerStarted'] = False
    # Property to get the
    bpy.types.Scene.maxFPS = bpy.props.IntProperty(name = "maxFPS", soft_min = 10, soft_max = 100)
    bpy.context.scene['maxFPS'] = defaultTimerHz
    _timer = None
    _maxFPS = defaultTimerHz

    def execute(self, context):
        logger.info('Starting Timer')
        wm = context.window_manager
        self._timer = wm.event_timer_add(1/self._maxFPS, context.window)
        bpy.context.scene['globalTimerStarted'] = True
        wm.modal_handler_add(self)
        return {'RUNNING_MODAL'}

    def modal(self, context, event):
        if event.type in {'ESC'}:
            return self.cancel(context)
        if event.type == 'TIMER':
            if self._maxFPS != bpy.context.scene['maxFPS']:
                #Add new timer
                wm = context.window_manager
                wm.event_timer_remove(self._timer)
                self._maxFPS = bpy.context.scene['maxFPS']
                self._timer = wm.event_timer_add(1/self._maxFPS, context.window)

        return {'PASS_THROUGH'}

    def cancel(self,context):
        logger.info('Stopping Timer')
        if self._timer:
            wm = context.window_manager
            wm.event_timer_remove(self._timer)

        bpy.context.scene['globalTimerStarted'] = False
        return {'CANCELLED'}

    @classmethod
    def poll(cls, context):
        return not bpy.context.scene['globalTimerStarted']


class EvaDebug(bpy.types.Operator):
    """Eva Debug Control"""
    bl_idname = "eva.debug"
    bl_label = "Eva Debug"

    action = bpy.props.StringProperty()

    # register some helper bpy props for dev purposes
    bpy.types.Scene.evaFollowMouse = bpy.props.BoolProperty(name = "evaFollowMouse")
    bpy.context.scene['evaFollowMouse'] = False

    bpy.types.Scene.evaFPS = bpy.props.IntProperty(name = "evaFPS", soft_min = 10, soft_max = 60)
    bpy.context.scene['evaFPS'] = 0

    bpy.types.Scene.evaEmotion = bpy.props.StringProperty(name = "evaEmotion")
    bpy.context.scene['evaEmotion'] = "{'happy': {'magnitude': 0.9, 'duration': 10}}"

    def execute(self, context):
        from . import commands
        logger.info(eval(self.action))
        return {'FINISHED'}



class BLPlayback(bpy.types.Operator):
    """Playback Control"""
    bl_label = "Start Animation"
    bl_idname = 'wm.animation_playback'

    bpy.types.Scene.animationPlaybackActive = bpy.props.BoolProperty( name = "animationPlaybackActive", default=False)
    bpy.context.scene['animationPlaybackActive'] = False
    bpy.context.scene['keepAlive'] = True

    timeList = []

    def modal(self, context, event):
        if event.type in {'ESC'}:
            return self.cancel(context)

        if event.type == 'TIMER':

            eva = bpy.evaAnimationManager

            # compute fps
            context.scene['evaFPS'] = fps = self.computeFPS(context)
            timeScale = framerateHz/fps
            time = self.timeList[-1]
            dt = self.timeList[-1] - self.timeList[-2] if len(self.timeList) > 1 else 0

            if bpy.context.scene['evaFollowMouse']:
                # compute mouse pos
                normalX = (event.mouse_region_x - 500) /1000
                normalY = (event.mouse_region_y - 500)/1000

                eyeLoc = eva.eyeTargetLoc.target
                eyeLoc[0] = -normalX * 0.6
                eyeLoc[2] = normalY * 0.5

                headLoc = eva.headTargetLoc.target
                headLoc[0] = -normalX * 0.2
                headLoc[2] = normalY * 0.2
            # update NLA based gestures
            gestures = eva.gesturesList[:]  	# prevent in-situ removal while iterating bug
            for gesture in gestures:
                # Leave strrip time for backward compatable with blender with version lower 2.75
                gesture.stripRef.strip_time += gesture.speed * timeScale
                gesture.strip_time_kfp.co[1] += gesture.speed * timeScale
                if gesture.stripRef.strip_time > gesture.duration:
                    if gesture.repeat > 1:
                        gesture.repeat -= 1
                        gesture.stripRef.strip_time = 0
                        gesture.strip_time_kfp.co[1] = 0
                    else:
                        eva._deleteGesture(gesture)
            # update visemes
            visemes = eva.visemesList[:]
            for viseme in visemes:

                # magnitude is a blendedNum.Trajectory and will internally take
                # care of the time it needs to activate.
                viseme.magnitude.blend(time, dt)

                if viseme.magnitude.is_done:
                    eva._deleteViseme(viseme)
                else:
                    # update action
                    viseme.stripRef.influence = viseme.magnitude.current
                    viseme.influence_kfp.co[1] = viseme.magnitude.current

            # update eye and head blending
            headControl = eva.bones["head_target"]
            eyeControl = eva.bones["eye_target"]

            # apply actuators
            eva.actuatorManager.tick(time, dt)

            eva.headTargetLoc.blend(time, dt)
            eva.eyeTargetLoc.blend(time, dt)
            eva.headRotation.blend(time, dt)

            head_loc = eva.headTargetLoc.current
            head_loc[0] = -head_loc[0]
            head_loc_old = eva.face_target
            headControl.location = head_loc
            eye_loc = eva.eyeTargetLoc.current
            eye_loc[1] = -eye_loc[1]
            eyeControl.location = eye_loc
            if abs(head_loc[0] + head_loc_old[0]) > 0.01:
                print(abs(head_loc[0] + head_loc_old[0]))
                bpy.data.scenes["Scene"].actuators.ACT_saccade.PARAM_scale = 0
            else:
                bpy.data.scenes["Scene"].actuators.ACT_saccade.PARAM_scale = 1


            scale = 0 if abs(head_loc[0] - head_loc_old[0]) else 1
            # Rotation
            headControl.rotation_euler[1] = eva.headRotation.current
            # udpate emotions
            for emotion in eva.emotionsList:
                emotion.magnitude.blend(time, dt)

                control = eva.bones['EMO-'+emotion.name]

                if emotion.magnitude.is_done:
                    eva.emotionsList.remove(emotion)
                    control['intensity'] = 0.0
                else:
                    control['intensity'] = emotion.magnitude.current


            if bpy.context.scene['keepAlive']:
                # Take care of Cycles
                # Ensure the strip is looping and sync strip props with cycle props.
                for cycle in eva.cyclesSet:
                    if cycle.name not in [gesture.name for gesture in eva.gesturesList]:
                        # Strip finished, create new one
                        eva.newGesture(cycle.name, repeat=10, speed=cycle.rate, magnitude=cycle.magnitude)
                    else:
                        # Update strip properties
                        for gesture in eva.gesturesList:
                            if gesture.name == cycle.name:
                                gesture.stripRef.influence = cycle.magnitude
                                gesture.speed = cycle.rate
                                gesture.stripRef.mute = False
            else:
                for cycle in eva.cyclesSet:
                    for gesture in eva.gesturesList:
                        if gesture.name == cycle.name:
                            gesture.stripRef.mute = True
            # Apply animation mode and shapekeys
            # Check if the animations mode was changed and updates it
            eva.changeMode()
            # Apply queued shapekeys
            eva.applyShapeKeys()
            # force update
            bpy.data.scenes['Scene'].frame_set(1)

        return {'PASS_THROUGH'}


    def execute(self, context):
        logger.info('Starting Playback')
        wm = context.window_manager
        wm.modal_handler_add(self)
        bpy.context.scene['animationPlaybackActive'] = True
        if not bpy.context.scene['globalTimerStarted']:
            bpy.ops.wm.global_timer()
        return {'RUNNING_MODAL'}


    def cancel(self, context):
        logger.info('Stopping Playback')
        bpy.evaAnimationManager.terminate()
        bpy.context.scene['animationPlaybackActive'] = False
        return {'CANCELLED'}


    def computeFPS(self, context):
        # record
        self.timeList.append(time.time())

        # trim
        if len(self.timeList)> 100:
            self.timeList = self.timeList[-100:]

        # compute
        counter = 0
        for timestamp in self.timeList:
            if timestamp > (time.time() - 1):
                counter += 1

        return counter


    @classmethod
    def poll(cls, context):
        return not bpy.context.scene['animationPlaybackActive']



def register():
    bpy.utils.register_class(BLPlayback)
    bpy.utils.register_class(EvaDebug)
    bpy.utils.register_class(BLGlobalTimer)

def unregister():
    bpy.utils.unregister_class(BLPlayback)
    bpy.utils.unregister_class(EvaDebug)
    bpy.utils.unregister_class(BLGlobalTimer)


def refresh():
    try:
        register()
    except Exception as E:
        unregister()
        register()
