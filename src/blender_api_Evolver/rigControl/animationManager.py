# AnimationManager is the primary datastore for the various paramters
# that define the Eva character.

from .actuators import ActuatorManager

from . import actuators
from . import blendedNum
from .blendedNum.plumbing import Pipes, Wrappers
from .helpers import *

import math
import bpy
import random
import time
import imp
import pdb
from mathutils import Vector
import logging

#import GA_methods
from GA_methods import IGA
from .commands import EvaAPI

debug = True
logger = logging.getLogger('hr.blender_api.rigcontrol.animationmanager')

class AnimationManager():

    def __init__(self):
        logger.info('Starting AnimationManager singleton')

        # Gesture params
        self.gesturesList = []
        self.gesturesList_IGA = []
        self.emotionsList = []
        self.visemesList = []
        self.cyclesSet = set()


        # Start default cycles
        self.setCycle('CYC-normal', rate=1.0, magnitude=1.0, ease_in=0.0)
        self.setCycle('CYC-breathing', rate=1.0, magnitude=1.0, ease_in=0.0)
        self.setCycle('CYC-normal-saccades', rate=1.0, magnitude=1.0, ease_in=0.0)


        # Scale for Blender coordinates 1 BU in m.
        self.scale = 0.25
        # Distance between eyes target and 0 point in the head in BU
        self.eyes_distance = 0.32
        # Target minimum distance in m
        self.min_distance  = 0.1
        # Face target offset in BU
        # -4 for Sophia 1.0, -2 for blender only
        self.face_target_offset = -4
        # Eye_target distance in BU from 0 point
        # -4 for Sophia 1.0 -2 for blender rig
        self.eye_target_offset = -4


        # Head and Eye tracking parameters
        self.headTargetLoc = blendedNum.LiveTarget([0,0,0], transition=Wrappers.wrap([
                Pipes.exponential(3.5),
                Pipes.moving_average(window=0.4)],
                Wrappers.in_spherical(origin=[0, self.face_target_offset, 0], radius=4)
        ))
        self.eyeTargetLoc = blendedNum.LiveTarget([0,0,0], transition=Wrappers.wrap(
            Pipes.linear(speed=3),
            Wrappers.in_spherical(origin=[0, self.eye_target_offset, 0], radius=4)
        ))

        self.actuatorManager = ActuatorManager()

        # Internal vars
        self._time = 0
        self.nextTrigger = {}

        # global access
        self.deformObj = bpy.data.objects['deform']
        self.bones = bpy.data.objects['control'].pose.bones

        # Camera location. See issue #25 in github.
        # Basic assumptions:
        # 1) blender viewport is about 80cm from me.
        # 2) Eva is even with or just a little behind the viewport.
        # 3) Eva's head is 14cm wide. (median width for female head)
        # 4) One "blender unit" (BU) is 25 centimeters.
        # This means that, with a camera field-of-view of 10 degrees
        # she should just fill the viewport.  That's becasue the half
        # angle is given by  arctan(7/80) = 0.0873 radians = 5 degrees
        # or equivalently, a whole angle of 10 degrees (0.175 radians).
        # Then, with this camera FOV, the camera has to be positioned
        # at -3.2 "blender units" (80cm) to get her face to actually
        # fill the viewport, width-wise.
        bpy.data.cameras["Camera.001"].angle = 0.175
        bpy.data.objects["Camera.001"].location = [0, -3.2, 0.75]

        # 90 degrees, exactly.
        bpy.data.objects["Camera.001"].rotation_euler = [1.570796, 0.0, 0.0]



        self.availableVisemes = []
        for action in bpy.data.actions:
            if action.name.startswith('VIS-'):
                self.availableVisemes.append(action)

        if debug:
            imp.reload(actuators)


    # Show all attributes
    def __repr__(self):
        string = ""
        for attr, value in sorted(self.__dict__.items()):
            if not attr.startswith('_'):
                string += str(attr) + ": " + str(value) + "\n"
        return string


    def newGesture(self, name, repeat = 1, speed=1, magnitude=0.5, priority=1):
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
        g = Gesture(name, newTrack, newStrip, duration=duration, speed=speed, \
             magnitude=magnitude, priority=priority, repeat=repeat, strip_time_kfp=strip_time_kfp)
        self.gesturesList.append(g)

    def newGesture_IGA(self, name, repeat = 1, speed=1, magnitude=0.5, priority=1):
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
        g = Gesture(name, newTrack, newStrip, duration=duration, speed=speed, \
             magnitude=magnitude, priority=priority, repeat=repeat, strip_time_kfp=strip_time_kfp)

        self.gesturesList_IGA.append(g)

# IGA related
    def _deleteGesture_IGA(self, gesture):
        ''' internal use only, stops and deletes a gesture'''
        # remove from list
        #bpy.evaAnimationManager.gesturesList_IGA.remove(bpy.evaAnimationManager.gesturesList_IGA[0])
         
        self.gesturesList_IGA.remove(gesture)

        # remove from Blender
        self.deformObj.animation_data.nla_tracks.remove(gesture.trackRef)
    
    

    def _deleteGesture(self, gesture):
        ''' internal use only, stops and deletes a gesture'''
        # remove from list
        self.gesturesList.remove(gesture)

        # remove from Blender
        self.deformObj.animation_data.nla_tracks.remove(gesture.trackRef)


    def setEmotion(self, emotionDict):
        '''Set the emotional state of the character.'''
        for emotionName, data in emotionDict.items():
            try:
                control = self.bones['EMO-'+emotionName]
            except KeyError:
                logger.error('Cannot set emotion. No bone with name {}'.format(emotionName))
                continue
            else:
                found = False
                for emotion in self.emotionsList:
                    if emotionName == emotion.name:
                        # update magnitude
                        num = emotion.magnitude
                        num.keyframes = []
                        num.add_keyframe(target=data['magnitude'], transition=(0, Pipes.linear(2)))
                        num.add_keyframe(target=0.0, transition=(0, Pipes.exponential(0.8/data['duration'])))
                        found = True

                if not found:
                    num = blendedNum.Trajectory(0)
                    num.add_keyframe(target=data['magnitude'], transition=[
                        (0, Pipes.linear(2)), (1, Pipes.moving_average(0.2))])
                    num.add_keyframe(target=0.0, transition=(0, Pipes.exponential(0.8/data['duration'])))
                    emotion = Emotion(emotionName, magnitude=num)
                    self.emotionsList.append(emotion)


    def newViseme(self, vis, duration=0.5, rampInRatio=0.1, rampOutRatio=0.8, startTime=0):
        '''Perform a new viseme'''
        # vis should be a string found in self.availableVisemes
        # duration is time to stay in this viseme in seconds, (including ramp time).
        # ramp*Ratio are the percentage of time the animation transitions in.
        # 	i.e. In=0, Out=0 would be no blending
        # 	In = 0.5, Out=0.5 would be maximum blending in and out
        # startTime is the starting time for the viseme in seconds relative to now.

        action = None
        for viseme in self.availableVisemes:
            if vis in viseme.name:
                action = viseme
                break

        if not action:
            raise TypeError('Visemee \"' + vis + '\" is not known')

        # Check value for sanity
        checkValue(duration, 0, 10)
        checkValue(rampInRatio, 0, 0.9)
        checkValue(rampOutRatio, 0, 0.9)
        checkValue(rampInRatio+rampOutRatio, 0, 1.0)

        # Create NLA track
        newTrack = self.deformObj.animation_data.nla_tracks.new()
        newTrack.name = action.name

        # Create strip
        newStrip = newTrack.strips.new(name=action.name, start=1, action=action)
        newStrip.blend_type = 'ADD'
        newStrip.use_animated_influence = True
        newStrip.influence = 0
        # Get influence function
        f = newStrip.fcurves.items()[0][1]
        # Point at 1st frame
        influence_kfp = f.keyframe_points.insert(1,0,{'FAST'})
        # Create object and add to list
        v = Viseme(action.name, newTrack, newStrip, duration, rampInRatio, rampOutRatio, startTime, influence_kfp)
        self.visemesList.append(v)

        return True


    def setCycle(self, name, rate, magnitude, ease_in):
        if magnitude == 0:
            # Remove cycle
            toRemove = [cycle for cycle in self.cyclesSet if cycle.name == name]
            for cycle in toRemove:
                self.cyclesSet.remove(cycle)

            toRemove = [gesture for gesture in self.gesturesList if gesture.name == name]
            for gesture in toRemove:
                self._deleteGesture(gesture)
            return 0
        else:
            # Check value for sanity
            checkValue(rate, 0.1, 10)
            checkValue(magnitude, 0, 1)

            # Add or update cycle with new parameters
            newCycle = Cycle(name, rate, magnitude, ease_in)
            self.cyclesSet.discard(newCycle)
            self.cyclesSet.add(newCycle)

    #========== define unique cycles that don't conform to name rate magnitude parameter ============
    def setBlinkRandomly(self,interval_mean,interval_variation):
        '''enable if necessary and update the blink rate of the artistic actuator'''

        if bpy.data.scenes["Scene"].actuators.ACT_blink_randomly.HEAD_PARAM_enabled == False:
           bpy.data.scenes["Scene"].actuators.ACT_blink_randomly.HEAD_PARAM_enabled = True
           print("enabled blinking")
        checkValue(interval_mean,0.5,10)
        checkValue(interval_variation,0.0,interval_mean)
        print('changing blink rate to ',interval_mean)
        bpy.data.scenes["Scene"].actuators.ACT_blink_randomly.PARAM_interval_mean=interval_mean
        bpy.data.scenes["Scene"].actuators.ACT_blink_randomly.PARAM_interval_variation=interval_variation
        # if reset delete and restart actuator

    def setSaccade(self,interval_mean,interval_variation,paint_scale,eye_size,eye_distance,mouth_width,mouth_height,weight_eyes,weight_mouth):
        '''enable if necessary and update the saccade rate of the artistic actuator'''
        if bpy.data.scenes["Scene"].actuators.ACT_saccade.HEAD_PARAM_enabled == False:
            bpy.data.scenes["Scene"].actuators.ACT_saccade.HEAD_PARAM_enabled = True
            print("enabled saccades")
        checkValue(interval_mean,0.1,5)
        checkValue(interval_variation,0.0,interval_mean)

        bpy.data.scenes["Scene"].actuators.ACT_saccade.PARAM_interval_mean=interval_mean
        bpy.data.scenes["Scene"].actuators.ACT_saccade.PARAM_interval_variation=interval_variation
        bpy.data.scenes["Scene"].actuators.ACT_saccade.PARAM_paint_scale=paint_scale
        # heat map parameters
        bpy.data.scenes["Scene"].actuators.ACT_saccade.PARAM_eye_size=eye_size
        bpy.data.scenes["Scene"].actuators.ACT_saccade.PARAM_eye_distance=eye_distance
        bpy.data.scenes["Scene"].actuators.ACT_saccade.PARAM_mouth_width=mouth_width
        bpy.data.scenes["Scene"].actuators.ACT_saccade.PARAM_mouth_height=mouth_height
        bpy.data.scenes["Scene"].actuators.ACT_saccade.PARAM_weight_mouth=weight_mouth
        bpy.data.scenes["Scene"].actuators.ACT_saccade.PARAM_weight_eyes=weight_eyes



        # if reset delete and restart actuator
    def _deleteViseme(self, viseme):
            ''' internal use only, stops and deletes a viseme'''
            # remove from list
            self.visemesList.remove(viseme)

            # remove from Blender
            self.deformObj.animation_data.nla_tracks.remove(viseme.trackRef)


    def coordConvert(self, loc, currbu, offset=0):
        '''Convert coordinates from meters to blender units. The
        coordinate frame used here is y is straight-ahead, x is th the
        right, and z is up.
        '''


        # Set minimum distance
        loc[1] = max(loc[1], self.eyes_distance*self.scale+self.min_distance)

        # Convert from meters to 'blender-units'
        locBU = m2bu(loc, self.scale)

        # Add offset if any
        locBU[1] += offset

        # Compute distance from previous eye position
        distance = computeDistance(locBU, currbu)

        return locBU


    def setFaceTarget(self, loc):
        '''Set the target used by eye and face tracking.'''

        locBU = self.coordConvert(loc, self.eyeTargetLoc.current, self.face_target_offset)

        self.headTargetLoc.target = locBU

        # Change offset for the eyes
        locBU[1] = locBU[1] - self.face_target_offset + self.eye_target_offset

        # Move eyes too, slowly
        self.eyeTargetLoc.transition = Wrappers.wrap([
                Pipes.linear(speed=0.5),
                Pipes.stick(window=0.5),
                Pipes.moving_average(window=0.1)],
            Wrappers.in_spherical(origin=[0, self.eye_target_offset, 0], radius=4))
        self.eyeTargetLoc.target = locBU

    def setGazeTarget(self, loc):
        '''Set the target used for eye tracking only.'''

        locBU = self.coordConvert(loc, self.eyeTargetLoc.current, self.eye_target_offset)

        self.eyeTargetLoc.transition = Wrappers.wrap(
            Pipes.linear(speed=3),
            Wrappers.in_spherical(origin=[0, self.eye_target_offset, 0], radius=4)
        )
        self.eyeTargetLoc.target = locBU

    def setViseme(self):
        pass


    def terminate(self):
        '''House-keeping at the end of the run'''
        # remove all leftover gestures
        for gesture in self.gesturesList:
            self.deformObj.animation_data.nla_tracks.remove(gesture.trackRef)

        self.gesturesList = []

        # reset pose
        bpy.context.scene.objects.active = self.deformObj
        try:
            bpy.ops.pose.transforms_clear()
        except:
            bpy.ops.object.posemode_toggle()
            bpy.ops.pose.transforms_clear()


    def randomFrequency(self, name, hz):
        '''Returns a random true/false based on a hertz value as input'''
        now = time.time()
        try:
            success = now > self.nextTrigger[name]
        except KeyError:
            success = True

        if success:
            # prevents div by 0
            hz = max(hz, 0.0001)
            # sigma is hard coded to 1/5 the mu
            self.nextTrigger[name] = now + random.gauss(1.0/hz, 0.2/hz)
        return success

#*******************************************************************************************************
    def Generator_uniform(self, population, popn_weighted, popn_range, mutation_weight, selected_actions):      
        #weight = int((len(popn)* weight))
        IGA.childGenerator_uniformIGA(IGA, population, popn_weighted, popn_range, mutation_weight, selected_actions)

    def Generator_cutandsplice(self, population, popn_weighted, popn_range, mutation_weight, selected_actions):
        #weight = int((len(popn)* weight))
        IGA.childGenerator_cutandspliceIGA(IGA, population, popn_weighted, popn_range, mutation_weight)

#*******************************************************************************************************
    def Generator_blend(self, population, popn_weighted, popn_range, mutation_weight, selected_actions):      
        #weight = int((len(popn)* weight))
        IGA.childGenerator_blendIGA(IGA, population, popn_weighted, popn_range, mutation_weight, selected_actions)

#*******************************************************************************************************       
class Emotion():
    '''Represents an emotion'''
    def __init__(self, name, magnitude):
        self.name = name
        self.magnitude = magnitude
        self.priority = 0


class Gesture():
    '''Represents a blender actions'''
    def __init__(self, name, track, strip, duration, speed, magnitude, priority, repeat, strip_time_kfp):
        self.name = name
        self.duration = duration
        self.magnitude = magnitude
        self.speed = speed
        self.priority = priority
        self.repeat = repeat

        self.trackRef = track
        self.stripRef = strip

        self.strip_time_kfp = strip_time_kfp

class Viseme():
    '''Represents a Viseme'''
    def __init__(self, vis, track, strip, duration, rampInRatio, rampOutRatio, startTime, influence_kfp):
        self.vis = vis
        self.trackRef = track
        self.stripRef = strip
        self.duration = duration  		# duration of animation in seconds
        self.influence_kfp = influence_kfp  # Influence keyframe point to change influence
        # startTime - seconds to wait before playing this Viseme
        # rampInRatio - percentage of time spent blending in
        # rampOutRatio - percentage of time spent blending out

        # Convert time percentages to transition speeds
        speedIn = 1/(max(rampInRatio * duration, 0.01))
        speedOut = 1/(max(rampOutRatio * duration, 0.01))

        # Calculate when to start transitioning out
        startOut = (1-rampOutRatio) * duration + startTime

        # Set up the trajectory for the magnitude
        num = blendedNum.Trajectory(0)
        num.add_keyframe(target=0.0, time=startTime, transition=(1, Pipes.moving_average(0.2)))
        num.add_keyframe(target=1.0, time=startOut, transition=(0, Pipes.linear(speedIn)))
        num.add_keyframe(target=0.0, transition=(0, Pipes.linear(speedOut)))
        self.magnitude = num # normalized amplitutde

class Cycle():
    ''' Represents a cyclic gesture, or 'soma' '''
    def __init__(self, name, rate, magnitude, ease_in):
        self.name = name
        self.rate = rate
        self._magnitude = magnitude
        self.ease_in = ease_in
        self.time_started = time.time()

    # Magnitude based on ease_in time
    @property
    def magnitude(self):
        if self.ease_in > 0.1:
            ease = min(1, (time.time() - (self.time_started))/self.ease_in)
            return self._magnitude * ease
        else:
            return self._magnitude

    @magnitude.setter
    def magnitude(self,value):
        self._magnitude = value

    def __eq__(self, other):
        return self.name == other.name

    def __hash__(self):
        return hash(self.name)

def init():
    '''Create AnimationManager singleton and make it available for global access'''
    if hasattr(bpy, 'evaAnimationManager'):
        logger.info('Skipping Singleton instantiation')
    else:
        bpy.evaAnimationManager = AnimationManager()

    
