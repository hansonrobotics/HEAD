import random
import numpy as np
from rigControl import actuators
from rigControl.actuators import sleep

try:
    import bpy
except ImportError:
    # Mock bpy package to avoid errors on bpy.props.FloatProperty and similar
    # calls when importing this package not from Blender.
    from unittest import mock
    bpy = mock.Mock()

__doc__ = """
Functions in this file marked with the "@actuators.new" decorator will show up as
new actuators in Blender GUI and ROS interface.

Functions without this decorator can be safely created in this file if, for
example, the actuators need some "helper" functions.
"""

@actuators.new
def saccade(self):
    probability_map = FaceProbabilityMap(self)

    # Register GUI-controllable parameters
    intervalMean, intervalVariation, paintScale = (
        self.add_parameter(prop) for prop in [
            bpy.props.FloatProperty(name='interval mean', min=0.0, max=3.0, default=1.0),
            bpy.props.FloatProperty(name='interval variation', min=0.0, max=1.0, default=0.5),
            bpy.props.FloatProperty(name='paint scale', min=0.0, max=2.0, default=1.0)
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
        # Get the current probability matrix
        mat = probability_map.tick()

        # Calculate the time to the next saccade
        interval = intervalMean.val + unitgauss * intervalVariation.val
        nexttime = lasttime + interval

        if time >= nexttime:
            # Calculate new eye offset
            pos = np.random.choice(mat.size, p=mat.flat)
            pos = ((pos % mat.shape[0] - mat.shape[0]/2) * paintScale.val/mat.shape[0],
                   (pos // mat.shape[0] - mat.shape[1]/2) * paintScale.val/mat.shape[1])

            offset = [0,0,0]
            offset[0] = pos[0] # Horizontal
            offset[1] = 0
            offset[2] = pos[1] # Verticall

            # Generate new random factor and store current time for the next saccade.
            unitgauss = random.gauss(0, 1)
            lasttime = time

        # Apply offset to eye position
        eyeTargetLoc.target_add(offset)

        # Yield execution until next frame
        time, dt = yield


class FaceProbabilityMap:
    """ Asks the Actuator Manager to draw a face in the UI made of circles and
    ellipses. The image can then be updated via parameter sliders. """

    def __init__(self, actuator):
        param_list = [
            ('eyeSize', bpy.props.FloatProperty(name='eye size', min=1.0, max=50, default=20)),
            ('eyeDist', bpy.props.FloatProperty(name='eye distance', min=0.0, max=100, default=40)),
            ('mouthWidth', bpy.props.FloatProperty(name='mouth width', min=1.0, max=100, default=50)),
            ('mouthHeight', bpy.props.FloatProperty(name='mouth height', min=1.0, max=100, default=30)),
            ('eyeWeight', bpy.props.FloatProperty(name='weight eyes', min=0.0, max=1.0, default=0.5)),
            ('mouthWeight', bpy.props.FloatProperty(name='weight mouth', min=0.0, max=1.0, default=0.5))
        ]
        self.params = {}
        for name, prop in param_list:
            self.params[name] = actuator.add_parameter(prop)

        self.constant_sum_tick = constant_sum(self.params['eyeWeight'], self.params['mouthWeight'])

        self.img_size = (100, 100)
        self.img = actuator.add_image('probabilities', self.img_size)

    def tick(self):
        self.constant_sum_tick()

        eyes_mat = Matrix.circle((70, 50 - self.params['eyeDist'].val/2), self.params['eyeSize'].val, self.img_size)
        eyes_mat += Matrix.circle((70, 50 + self.params['eyeDist'].val/2), self.params['eyeSize'].val, self.img_size)
        eyes_mat *= self.params['eyeWeight'].val / eyes_mat.sum()

        mouth_mat = Matrix.ellipse((20, 50), (self.params['mouthHeight'].val, self.params['mouthWidth'].val), self.img_size)
        mouth_mat *= self.params['mouthWeight'].val / mouth_mat.sum()

        mat = eyes_mat + mouth_mat
        mat /= mat.sum()
        self.img.set_pixels(Matrix.color(mat/mat.max(), [1,1,0,1]))
        return mat


class Matrix:
    def circle(pos, radius, imgsize):
        """ Draw a circle on a new matrix of size 'imgsize'. """
        xx, yy = np.mgrid[:imgsize[0], :imgsize[1]]
        distances = (xx - pos[0]) ** 2 + (yy - pos[1]) ** 2
        radius2 = radius**2
        result = distances.clip(0, radius2)/radius2 * -1 + 1
        return result

    def ellipse(pos, size, imgsize):
        """ Draw an ellipse on a new matrix of size 'imgsize'. """
        xx, yy = np.mgrid[:imgsize[0], :imgsize[1]]
        ratio = size[0]/size[1]
        distances = (xx - pos[0]) ** 2 + ((yy - pos[1]) * ratio) ** 2
        ceiling = size[0]**2
        result = distances.clip(0, ceiling)/ceiling * -1 + 1
        return result

    def color(mat, color):
        """ Create an RGB matrix from a single-valued matrix and a color. """
        mat.shape = mat.shape + (1,)
        result = mat.repeat(4, axis=-1)
        mat.shape = mat.shape[:-1]
        result *= color
        return result.flatten()

def constant_sum(*params, target_sum=1.0):
    """ Takes ActuatorManager.Parameter instances and returns a 'tick' function
    that keeps their sum at 'target_sum' in case some of the values change. """
    lastvals = {(param, param.val) for param in params}
    def tick():
        # Pick parameters which were not updated since last tick
        vals = {(param, param.val) for param in params}
        unchanged = vals.intersection(lastvals)

        # Increase or decrease those parameters to fit the constant sum
        if len(unchanged) > 0:
            overflow = sum([val for _, val in vals]) - target_sum
            correction = -overflow/len(unchanged)
            for param, val in unchanged:
                param.val = val + correction

        lastvals.clear()
        lastvals.update(vals)
    return tick
