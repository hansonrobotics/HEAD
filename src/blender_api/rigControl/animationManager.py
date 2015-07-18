# AnimationManager is the primary datastore for the various paramters
# that define the Eva character.

from . import actuators
from .blendedNum import BlendedNum, Transitions, Wrappers
from .helpers import *

import math
import bpy
import random
import time
import imp
import pdb
from mathutils import Vector

debug = True


class AnimationManager():

    def __init__(self):
        print('Starting AnimationManager singleton')

        # Gesture params
        self.gesturesList = []
        self.emotionsList = []
        self.visemesList = []
        self.cyclesSet = set()


        # Start default cycles
        self.setCycle('CYC-normal', rate=1.0, magnitude=1.0, ease_in=0.0)
        self.setCycle('CYC-breathing', rate=1.0, magnitude=1.0, ease_in=0.0)


        # Scale for Blender coordinates 1 BU in m.
        self.scale = 0.25
        # Distance between eyes target and 0 point in the head in BU
        self.eyes_distance = 0.32
        # Target minimum distance in m
        self.min_distance  = 0.1
        # Face target offset in BU
        self.face_target_offset = -4
        # Eye_target distance in BU from 0 point
        self.eye_target_offset = -4


        # Head and Eye tracking parameters
        self.headTargetLoc = BlendedNum([0,0,0], transition=Wrappers.wrap(
            Transitions.chain(Transitions.linear(speed=0.5),
                              Transitions.moving_average(duration=0.6)),
            Wrappers.in_spherical(origin=[0, self.face_target_offset, 0])
        ))
        self.eyeTargetLoc = BlendedNum([0,0,0], transition=Wrappers.wrap(
            Transitions.linear(speed=3),
            Wrappers.in_spherical(origin=[0, self.eye_target_offset, 0])
        ))


        # Autonomous (unconscious) behavior parameters
        self.eyeDartRate = 1.0
        self.eyeWander = 1.0
        self.blinkRate = 0.0
        self.blinkDuration = 0.0

        # Emotional parameters
        self.swiftness = 1.0
        self.shyness = 1.0
        self.idle = 0.0

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


    def keepAlive(self, alive):
        '''Called every frame, used to dispatch animation actuators'''
        if alive:
            self.idle += 1.0

            for cycle in self.cyclesSet:
                actuators.doCycle(self, cycle)

            # if True and self.randomFrequency('dart', self.eyeDartRate):
            #     actuators.eyeSaccades(self, self.eyeWander)

            if True and self.randomFrequency('blink', self.blinkRate):
                actuators.blink(self, self.blinkDuration)

            # if True and self.randomFrequency('headTargetLoc', 1):
            #     actuators.headDrift(self)

            if True and self.randomFrequency('emotionJitter', 20):
                actuators.emotionJitter(self)
        else:
            for cycle in self.cyclesSet:
                for gesture in self.gesturesList:
                    if gesture.name == cycle.name:
                        gesture.stripRef.mute = True

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
                print('Cannot set emotion. No bone with name ', emotionName)
                continue
            else:
                found = False
                for emotion in self.emotionsList:
                    if emotionName == emotion.name:
                        # update magnitude
                        emotion.magnitude.target = data['magnitude']
                        emotion.duration = data['duration']
                        found = True

                if not found:
                    emotion = Emotion(emotionName, magnitude = BlendedNum(data['magnitude'], steps = 10, smoothing = 10), duration = data['duration'])
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


    def _deleteViseme(self, viseme):
            ''' internal use only, stops and deletes a viseme'''
            # remove from list
            self.visemesList.remove(viseme)

            # remove from Blender
            self.deformObj.animation_data.nla_tracks.remove(viseme.trackRef)


    def coordConvert(self, loc, currbu, offset= 0):
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

        # Behavior: if the point being looked at changed
        # significantly, then micro-blink.

        if self.randomFrequency('blink', 20):
            if (distance + abs(offset)) > 7.9:
                self.newGesture('GST-blink', priority=1)
            elif (distance + abs(offset)) > 6.5:
                self.newGesture('TRN-waitBlink', priority=1)

        return locBU


    def setFaceTarget(self, loc):
        '''Set the target used by eye and face tracking.'''

        locBU = self.coordConvert(loc, self.eyeTargetLoc.current, self.face_target_offset)
        self.headTargetLoc.target.base = locBU


        # Change offset for the eyes
        locBU[1] = locBU[1] - self.face_target_offset + self.eye_target_offset

        # Move eyes too, slowly
        self.eyeTargetLoc.transition = Wrappers.wrap(
            Transitions.chain(Transitions.linear(speed=0.5),
                              Transitions.moving_average(duration=0.6)),
            Wrappers.in_spherical(origin=[0, self.eye_target_offset, 0])
        )
        self.eyeTargetLoc.target.base = locBU

    def setGazeTarget(self, loc):
        '''Set the target used for eye tracking only.'''

        locBU = self.coordConvert(loc, self.eyeTargetLoc.current, self.eye_target_offset)

        self.eyeTargetLoc.transition = Wrappers.wrap(
            Transitions.linear(speed=3),
            Wrappers.in_spherical(origin=[0, self.eye_target_offset, 0])
        )
        self.eyeTargetLoc.target.base = locBU

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


class Emotion():
    '''Represents an emotion'''
    def __init__(self, name, magnitude, duration):
        self.name = name
        self.magnitude = magnitude
        self.duration = duration
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
        self.time = 0 - startTime 		# -time is scheduled for the future (seconds)
                                        # 0 is happening right away
                                        # +time is animation in progress (seconds)
        self.magnitude = BlendedNum(0, steps=2, smoothing=4) 	# normalized amplitude
        self.rampInRatio = rampInRatio 		# percentage of time spent blending in
        self.rampOutRatio = rampOutRatio 	# percentage of time spent blending out
        self.influence_kfp = influence_kfp  # Influence keyframe point to change influence


class Cycle():
    ''' Represents a cyclic gesture, or 'soma' '''
    def __init__(self, name, rate, magnitude, ease_in):
        self.name = name
        self.rate = rate
        self.magnitude = magnitude
        self.ease_in = ease_in

    def __eq__(self, other):
        return self.name == other.name

    def __hash__(self):
        return hash(self.name)


def init():
    '''Create AnimationManager singleton and make it available for global access'''
    if hasattr(bpy, 'evaAnimationManager'):
        print('Skipping Singleton instantiation')
    else:
        bpy.evaAnimationManager = AnimationManager()
