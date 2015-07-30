import random
# import bpy
from rigControl.actuators import Parameter, sleep, actuator


@actuator
def test():
    while True:
        yield from sleep(2.0)
        print("Look, I'm executing.")

#@actuator
def saccade():
    # Register GUI-controllable parameters
    frequencyMean = Parameter('frequency mean', min=0.0, max=5.0)
    frequencyVariation = Parameter('frequency variation', min=0.0, max=1.0)
    eyeWander = Parameter('eye wander', min=0.0, max=2.0)

    # Get the reference to eva's eye target.
    # eyeTargetLoc = bpy.evaAnimationManager.eyeTargetLoc

    while True:
        # Wait a random amount of time before the next saccade
        yield from sleep(frequencyMean.get() * random.gauss(1, frequencyVariation.get()))

        # Calculate new eye position
        newLoc = [0,0,0]
        newLoc[0] = random.gauss(eyeTargetLoc.current[0], eyeWander.get())
        newLoc[1] = eyeTargetLoc.current[1]
        newLoc[2] = random.gauss(eyeTargetLoc.current[2], eyeWander.get() * 0.5)

        # Override eye position
        eyeTargetLoc.current = newLoc


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

    index = randomSelect([micro, normal, relaxed, sleepy])
    action = ['GST-blink-micro', 'GST-blink', 'GST-blink-relaxed', 'GST-blink-sleepy']
    self.newGesture(action[index])

    # print(action[index])
