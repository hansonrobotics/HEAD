from collections import Sequence, deque
from copy import copy
from math import sqrt, atan, tan
import random

class Pipes:

    """
    Pipes define how blended numbers should reach the given target in time.

    Most pipes, on each yield, take a list of arbitrary length as their target
    and return a list of the same length.
    """

    @staticmethod
    def smooth(speed=1.0, smoothing=1.0):
        return [Pipes.linear(speed), Pipes.moving_average(smoothing)]

    @staticmethod
    def linear(speed):
        """ Speed is in units per second. """
        target, time, dt = yield None
        current = target
        while True:
            # Find the distance to 'target'
            displacement = [a - b for a, b in zip(target, current)]
            distance = sqrt(sum(a**2 for a in displacement))

            if distance > speed * dt:
                # Find the unit direction vector to 'target'
                direction = [a / distance for a in displacement]

                # Move 'current' to that direction in proportion to speed and dt.
                current = [a + b * speed * dt for a, b in zip(current, direction)]
            else:
                current = target

            target, time, dt = yield current

    @staticmethod
    def moving_average(window):
        """ Window is in seconds. """
        target, time, dt = yield None
        buffer = WeightBuffer()
        while True:
            buffer.append((target, dt))
            buffer.cut_to_fit(window)
            target, time, dt = yield buffer.weighted_mean()

    @staticmethod
    def exponential(alpha):
        """ Exponential decay or exponential moving average.
        Alpha defines the fraction of the distance to move towards the target
        in one second."""
        target, time, dt = yield None
        current = target
        while True:
            current = [b + alpha*(a-b)*dt for a,b in zip(target, current)]
            target, time, dt = yield current


    @staticmethod
    def stick(window, deviation=0.5, time_interval_func=lambda: random.uniform(0.2, 0.5)):
        """ Best works after a 'linear' pipe. Sticks onto target values for a
        period of time returned by the 'time_interval_func' argument.
        'window' defines the time in seconds for this pipe to ramp in and ramp out.
        'deviation' defines how much the output should deviate from the target,
         when the target is moving. """
        target, time, dt = yield None
        buffer = WeightBuffer()
        spacerandom = lambda sigma: [random.gauss(a, b) for a, b in
                                     zip(buffer.weighted_mean(), sigma)]
        nexttime = time
        while True:
            buffer.append((target, dt))
            buffer.cut_to_fit(window)

            buffer_variance = buffer.weighted_variance()
            buffer_deviation = [sqrt(a) for a in buffer_variance]
            total_buffer_deviation = sqrt(sum(buffer_variance))

            if time >= nexttime:
                nexttime = time_interval_func() + time
                old_target = spacerandom(
                    [(0.5*a + 0.5*total_buffer_deviation) * deviation
                     for a in buffer_deviation]
                )

            target, time, dt = yield old_target

    @staticmethod
    def circles():
        target, time, dt = yield None
        start = time
        buffer = WeightBuffer()
        while True:
            buffer.append((target, dt))
            buffer.cut_to_fit(1.0)
            radius = sum(buffer.weighted_variance()) * 8
            mean = buffer.weighted_mean()

            from math import sin, cos
            x, y = sin((time - start)*6) * radius, cos((time - start)*6) * radius
            target, time, dt = yield mean[0] + x, mean[1] + y

    @staticmethod
    def identity():
        target, time, dt = yield None
        while True:
            target, time, dt = yield target


class Wrappers:

    """
    Pipes that come in pairs and should be placed in front and at the end of an
    already existing pipeline.

    Useful for temporarily changing the coordinate system.
    """

    @staticmethod
    def in_spherical(origin, radius=3):
        x0, y0, z0 = origin

        def cartesian_to_spherical():
            target, _, _ = yield None
            while True:
                x, y, z = target
                x, y, z = x-x0, y-y0, z-z0
                yaw = atan(x / y)
                pitch = atan(z / y)
                target, _, _ = yield [yaw, pitch]

        def spherical_to_cartesian():
            target, _, _ = yield None
            while True:
                yaw, pitch = target
                y = radius / sqrt(1 + tan(yaw)**2 + tan(pitch)**2)
                x = y * tan(yaw)
                z = y * tan(pitch)
                target, _, _ = yield [x+x0, y+y0, z+z0]

        return cartesian_to_spherical(), spherical_to_cartesian()

    @staticmethod
    def wrap(pipes, wrapper):
        prepend_pipes, append_pipes = wrapper
        return [prepend_pipes, pipes, append_pipes]


class WeightBuffer(deque):
    """ Required element type: (List[float], float).
    The list being a value vector of arbitrary length and the last float being
    the weight of the element.

    These weights enable useful methods like 'weighted_mean' and 'cut_to_fit'.
    See their docstrings. """

    def weighted_mean(self):
        """ Returns the average vector taking weights into account. """
        return [buf._scalar_weighted_mean() for buf in self._by_dimension()]

    def weighted_variance(self):
        """ Returns the statistical variance vector taking weights into account. """
        return [buf._scalar_weighted_variance() for buf in self._by_dimension()]

    def cut_to_fit(self, capacity):
        """ Works similary to a circular buffer, but in terms of a fixed
        floating number weight capacity instead of a fixed number of elements.

        Trim the left side until the buffer matches the given 'capacity'.
        E.g. WeightBuffer([(..., 0.2), (..., 0.8), (..., 0.4)]).cut_to_fit(1.0)
        would result in WeightBuffer([(..., 0.6), (..., 0.4)])
        """
        overflow = sum(weight for val, weight in self) - capacity

        if overflow <= 0:
            return

        while overflow > 0:
            val, weight = self.popleft()
            overflow -= weight

        if overflow < 0:
            self.appendleft((val, abs(overflow)))

    def _scalar_weighted_mean(self):
        weighted_sum = sum(val * weight for val, weight in self)
        sum_of_weights = sum(weight for val, weight in self)
        if weighted_sum == 0:
            return 0 # Mitigate weighted_sum, sum_of_weights = 0, 0 cases.
        return weighted_sum / sum_of_weights

    def _scalar_weighted_variance(self):
        weighted_sum = sum(val * weight for val, weight in self)
        sum_of_weights = sum(weight for val, weight in self)
        if weighted_sum == 0:
            return 0 # Mitigate weighted_sum, sum_of_weights = 0, 0 cases.
        mean = weighted_sum / sum_of_weights
        variance = sum(weight * (val - mean)**2 for val, weight in self) / sum_of_weights
        return variance

    def _by_dimension(self):
        vectors, weights = zip(*self)
        for dimension in zip(*vectors):
            yield WeightBuffer(zip(dimension, weights))


class Sources:

    @staticmethod
    def constant(val):
        """ A target that always yields the same value. """
        val = _vectorize(val)
        while True:
            yield val

    class AdditiveTarget:

        """
        A target that can be modified permenantly (base property) or temporarily,
        a change that lasts only for the next frame (add function).

        Useful for blending several targets (animations) into one.
        """

        def __init__(self, base):
            self.base = _vectorize(base)
            self._accumulator = None

        @property
        def base(self):
            return _devectorize(self._base)

        @base.setter
        def base(self, val):
            """ A value that will not be cleared with the blend() method.
            Unlike add(), this method uses only the last value. """
            self._base = _vectorize(val)

        def add(self, val):
            """ Values that will be summed, added with 'base' and cleared on every
            call to blend(). """
            val = _vectorize(val)

            if self._accumulator == None:
                self._accumulator = val
            else:
                self._accumulator = [a + b for a, b in zip(self._accumulator, val)]

        def blend(self, time, dt):
            maxlen = max(len(vec) if isinstance(vec, Sequence) else 0
                         for vec in [self._accumulator, self._base])

            # If any of the vectors are None, fill them with zeros.
            base = self._base or [0] * maxlen
            accumulator = self._accumulator or [0] * maxlen

            # Clear the accumulator
            self._accumulator = None

            return [a + b for a, b in zip(accumulator, base)]

        def send(self, tupl):
            """ This function ducktypes the class for it to be interchangable with
            a python generator. """
            return self.blend(*tupl)


class Plumbing:

    def __init__(self, source=None, pipes=[]):
        self.source = source
        self.pipes = pipes

    def blend(self, time, dt):
        try:
            output = self.source.send((time, dt))
        except TypeError:
            # It appears the generator was expecting None.
            # First sent value has to be None for any non-initialized generator.
            self.source.send(None)
            output = self.source.send((time, dt))

        for pipe in self.pipes:
            try:
                output = pipe.send((output, time, dt))
            except TypeError:
                pipe.send(None)
                output = pipe.send((output, time, dt))
        return output


def _vectorize(value):
    if not isinstance(value, Sequence):
        return [value]
    else:
        return _flatten(value)

def _flatten(lst):
    result = []
    for item in lst:
        if isinstance(item, Sequence):
            result.extend(_flatten(item))
        else:
            result.append(item)
    return result

def _devectorize(vector):
    if len(vector) == 1:
        return vector[0]
    else:
        return copy(vector)
