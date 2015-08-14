import random
import numpy as np
import bpy
from rigControl.actuators import sleep, actuator

class Matrix:
    def circle(pos, radius, imgsize):
        xx, yy = np.mgrid[:imgsize[0], :imgsize[1]]
        distances = (xx - pos[0]) ** 2 + (yy - pos[1]) ** 2
        radius2 = radius**2
        result = distances.clip(0, radius2)/radius2 * -1 + 1
        return result

    def ellipse(pos, size, imgsize):
        xx, yy = np.mgrid[:imgsize[0], :imgsize[1]]
        ratio = size[0]/size[1]
        distances = (xx - pos[0]) ** 2 + ((yy - pos[1]) * ratio) ** 2
        ceiling = size[0]**2
        result = distances.clip(0, ceiling)/ceiling * -1 + 1
        return result

    def color(mat, color):
        mat.shape = mat.shape + (1,)
        result = mat.repeat(4, axis=-1)
        mat.shape = mat.shape[:-1]
        result *= color
        return result.flatten()

def constant_sum(*params):
    lastvals = {(param, param.val) for param in params}
    def tick():
        # Pick parameters which were not updated since last tick
        vals = {(param, param.val) for param in params}
        unchanged = vals.intersection(lastvals)

        # Increase or decrease those parameters to fit the constant sum
        if len(unchanged) > 0:
            overflow = sum([val for _, val in vals]) - 1
            correction = -overflow/len(unchanged)
            for param, val in unchanged:
                param.val = val + correction

        lastvals.clear()
        lastvals.update(vals)
    return tick

@actuator
def paint_face(self):
    img_size = (100, 100)
    img = self.add_image('probabilities', img_size)

    eyeSize, eyeDist, mouthWidth, mouthHeight, eyeWeight,  mouthWeight = (
        self.add_parameter(prop) for prop in [
            bpy.props.FloatProperty(name='eye size', min=1.0, max=50, default=20),
            bpy.props.FloatProperty(name='eye distance', min=0.0, max=100, default=40),
            bpy.props.FloatProperty(name='mouth width', min=1.0, max=100, default=50),
            bpy.props.FloatProperty(name='mouth height', min=1.0, max=100, default=30),
            bpy.props.FloatProperty(name='weight eyes', min=0.0, max=1.0, default=0.5),
            bpy.props.FloatProperty(name='weight mouth', min=0.0, max=1.0, default=0.5)
        ]
    )

    constant_sum_tick = constant_sum(eyeWeight, mouthWeight)
    while True:
        constant_sum_tick()

        eyes_mat = Matrix.circle((70, 50 - eyeDist.val/2), eyeSize.val, img_size)
        eyes_mat += Matrix.circle((70, 50 + eyeDist.val/2), eyeSize.val, img_size)
        eyes_mat *= eyeWeight.val / eyes_mat.sum()

        mouth_mat = Matrix.ellipse((20, 50), (mouthHeight.val, mouthWidth.val), img_size)
        mouth_mat *= mouthWeight.val / mouth_mat.sum()

        mat = eyes_mat + mouth_mat
        img.set_pixels(Matrix.color(mat/mat.max(), [1,1,0,1]))
        time, dt = yield

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
    self.add_parameter(bpy.props.StringProperty(name='interval',
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
