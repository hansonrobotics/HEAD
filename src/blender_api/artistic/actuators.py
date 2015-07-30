import random
import bpy
from rigControl.actuators import sleep, actuator


@actuator
def test(self):
    self.add_parameter(bpy.props.FloatProperty(name='my slider', min=0.0, max=5.0))
    while True:
        yield from sleep(2.0)
        print("Look, I'm executing.")

@actuator
def saccade(self):
    # Register GUI-controllable parameters
    eyeWander, intervalMean, intervalVariation = (
        self.add_parameter(prop) for prop in [
            bpy.props.FloatProperty(name='eye wander', min=0.0, max=1.0, default=0.3),
            bpy.props.FloatProperty(name='interval mean', min=0.0, max=3.0, default=1.0),
            bpy.props.FloatProperty(name='interval variation', min=0.0, max=1.0, default=0.5)
        ]
    )


    # Register GUI-displayable parameter
    interval = 0
    self.add_parameter(bpy.props.StringProperty(name="interval",
        get=lambda _: '{:0.3f}s'.format(interval)))

    # Yield execution to get current time
    time, dt = yield

    # Initialize some variables
    lasttime = time
    offset = [0,0,0]
    unitgauss = random.gauss(0, 1)

    # Get the reference to eva's eye target.
    eyeTargetLoc = bpy.evaAnimationManager.eyeTargetLoc

    while True:

        # Calculate the time to the next saccade
        #denominator = frequencyMean.val + unitgauss * frequencyVariation.val
        interval = intervalMean.val + unitgauss * intervalVariation.val
        nexttime = lasttime + interval

        if time >= nexttime:
            # Calculate new eye offset
            offset = [0,0,0]
            offset[0] = random.gauss(0, eyeWander.val) # Horizontal
            offset[1] = 0
            offset[2] = random.gauss(0, eyeWander.val * 0.5) # Vertical

            # Generate new random factor and store current time for the next saccade.
            unitgauss = random.gauss(0, 1)
            lasttime = time

        # Apply offset to eye position
        eyeTargetLoc.target.add(offset)

        # Yield execution until next frame
        time, dt = yield


def doCycle(self, cycle):
    if cycle.name not in [gesture.name for gesture in self.gesturesList]:
        # create new strip
        self.newGesture(cycle.name, repeat=10, speed=cycle.rate, magnitude=cycle.magnitude)
    else:
        # update strip property
        for gesture in self.gesturesList:
            if gesture.name == cycle.name:
                gesture.stripRef.influence = cycle.magnitude
                gesture.speed = cycle.rate
                gesture.stripRef.mute = False


def emotionJitter(self):
    for emotion in self.emotionsList:
        target = emotion.magnitude.target
        emotion.magnitude.target = random.gauss(target, target/20)


def headDrift(self):
    ''' applies random head drift '''
    loc = [0,0,0]
    loc[0] = random.gauss(self.headTargetLoc.target[0], self.headTargetLoc.target[0]/100)
    loc[1] = random.gauss(self.headTargetLoc.target[1], self.headTargetLoc.target[1]/100)
    loc[2] = random.gauss(self.headTargetLoc.target[2], self.headTargetLoc.target[2]/100)
    self.headTargetLoc.target = loc


def blink(self, duration):
    # compute probability
    micro = -(abs(duration+1)-1)
    normal = -(abs(duration+0)-1)
    relaxed = -(abs(duration-1)-1)
    sleepy = -(abs(duration-2)-1)

    micro = max(micro, 0)
    normal = max(normal, 0)
    relaxed = max(relaxed, 0)
    sleepy = max(sleepy, 0)

    index = 0 #randomSelect([micro, normal, relaxed, sleepy])
    action = ['GST-blink-micro', 'GST-blink', 'GST-blink-relaxed', 'GST-blink-sleepy']
    self.newGesture(action[index])

    # print(action[index])
