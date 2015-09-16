# This module sets up a modal operator in Blender to act
# as the animation playback service, and hosts other supporting test operators

# Frame rate for animation playback
framerateHz = 24

# System timer
defaultTimerHz = 48

import bpy
from .helpers import *

import pprint, time

class BLGlobalTimer(bpy.types.Operator):
    """Timer  Control"""
    bl_label = "Global Timer"
    bl_idname = 'wm.global_timer'
    # New property for global timer
    bpy.types.Scene.globalTimerStarted = bpy.props.BoolProperty( name = "globalTimerStarted", default=False)
    bpy.context.scene['globalTimerStarted'] = False
    # Property to get the
    bpy.types.Scene.maxFPS = bpy.props.IntProperty(name = "maxFPS", soft_min = 10, soft_max = 100)
    bpy.context.scene['maxFPS'] = defaultTimerHz
    _timer = None
    _maxFPS = defaultTimerHz

    def execute(self, context):
        print('Starting Timer')
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
        print('Stopping Timer')
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
        print(eval(self.action))
        return {'FINISHED'}



class BLPlayback(bpy.types.Operator):
    """Playback Control"""
    bl_label = "Start Animation"
    bl_idname = 'wm.animation_playback'

    bpy.types.Scene.animationPlaybackActive = bpy.props.BoolProperty( name = "animationPlaybackActive", default=False)
    bpy.context.scene['animationPlaybackActive'] = False

    timeList = []

    def modal(self, context, event):
        if event.type in {'ESC'}:
            return self.cancel(context)

        if event.type == 'TIMER':

            eva = bpy.evaAnimationManager

            # compute fps
            context.scene['evaFPS'] = fps = self.computeFPS(context)
            timeScale = framerateHz/fps

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
                gesture.stripRef.strip_time += gesture.speed * timeScale

                if gesture.stripRef.strip_time > gesture.duration:
                    if gesture.repeat > 1:
                        gesture.repeat -= 1
                        gesture.stripRef.strip_time = 0
                    else:
                        eva._deleteGesture(gesture)

            # update visemes
            visemes = eva.visemesList[:]
            for viseme in visemes:
                # wait to start
                if viseme.time < 0:
                    continue

                # remove if finished (and finalized)
                if viseme.time > viseme.duration*1.5:
                    eva._deleteViseme(viseme)
                    continue

                # ramp in from 0
                rampPoint = viseme.duration * viseme.rampInRatio
                if viseme.time <= rampPoint:
                    # compute ramp in factor
                    viseme.magnitude.target = viseme.time / rampPoint

                # ramp out to 0
                rampOutPoint = viseme.duration - viseme.duration*viseme.rampOutRatio
                if viseme.time >= rampOutPoint:
                    # compute ramp in factor
                    viseme.magnitude.target = 1.0 - (viseme.time - rampOutPoint) / (viseme.duration*viseme.rampOutRatio)

                # update action
                viseme.magnitude.blend()
                viseme.stripRef.influence = viseme.magnitude.current

                # update time
                viseme.time += (1/framerateHz)*timeScale

            # update eye and head blending
            headControl = eva.bones["head_target"]
            eyeControl = eva.bones["eye_target"]

            eva.headTargetLoc.blend()
            eva.eyeTargetLoc.blend()
            eye_loc = eva.eyeTargetLoc.current
            head_loc = eva.headTargetLoc.current
            head_loc[1] = -head_loc[1]
            head_loc[0] = -head_loc[0]
            eyeControl.location = eye_loc
            headControl.location = head_loc

            # udpate emotions
            for emotion in eva.emotionsList:
                control = eva.bones['EMO-'+emotion.name]
                control['intensity'] = emotion.magnitude.current
                emotion.duration -= timeScale
                emotion.magnitude.blend()

                if emotion.duration < 0:
                    emotion.magnitude._target *= 0.99

                    if emotion.magnitude.current < 0.1:
                        eva.emotionsList.remove(emotion)
                        control['intensity'] = 0.0


            # Read emotion parameters into eva
            eva.eyeDartRate = eva.deformObj.pose.bones['eye_dart_rate']['value']
            eva.eyeWander = eva.deformObj.pose.bones['eye_wander']['value']
            eva.blinkRate = eva.deformObj.pose.bones['blink_rate']['value']
            eva.blinkDuration = eva.deformObj.pose.bones['blink_duration']['value']

            # keep alive
            eva.keepAlive()

            # send ROS data

            # force update
            bpy.data.scenes['Scene'].frame_set(1)

        return {'PASS_THROUGH'}


    def execute(self, context):
        print('Starting Playback')
        wm = context.window_manager
        wm.modal_handler_add(self)
        bpy.context.scene['animationPlaybackActive'] = True
        if not bpy.context.scene['globalTimerStarted']:
            bpy.ops.wm.global_timer()
        return {'RUNNING_MODAL'}


    def cancel(self, context):
        print('Stopping Playback')
        bpy.evaAnimationManager.terminate()
        bpy.context.scene['animationPlaybackActive'] = False
        return {'CANCELLED'}


    def computeFPS(self, context):
        # record
        self.timeList.append(time.time())

        # trim
        if len(self.timeList)> 100:
            self.timeList[100:]

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
