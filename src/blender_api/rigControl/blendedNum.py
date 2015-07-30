from collections import deque, Sequence
from math import tan, atan, sqrt
from copy import copy
import random
from time import time as systime

class BlendedNum():
    def __init__(self, value, transition=None):
        value = _vectorize(value)

        self._current = value
        self._target = Target(value)

        self._transition = None
        self.transition = transition or Transitions.identity()

    @property
    def current(self):
        return _devectorize(self._current)

    @property
    def target(self):
        """ The target only has a getter to discourage overriding it.
        Use one of two methods to set the target: target.base or target.add()."""
        return self._target

    @target.setter
    def target(self, val):
        self._target.base = val

    def blend(self, time, dt):
        """ Updates the 'current' value and clears any modifications added to
        the 'target' with target.add(val) method (used by actuators,
        procedural animations). """
        target_val = self._target.clear()
        self._current = self.transition.send((target_val, time, dt))
        return self._current

    @property
    def transition(self):
        return self._transition

    @transition.setter
    def transition(self, val):
        if val == self._transition:
            return

        # Make the transition think it's been at the current position for 10 seconds
        # for initialization purposes.
        send_current = lambda: val.send((self._current, systime(), 10.0))

        try:
            send_current()
        except TypeError:
            # It appears the generator was expecting None.
            # First sent value has to be None for any non-initialized generator.
            val.send(None)
            send_current()

        self._transition = val

    def __repr__(self):
        return "<BlendedNum current={} target.base={}>".format(
            self._current, self._target._base)


class Transitions:

    @staticmethod
    def linear(speed):
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
    def moving_average(duration):
        """ Duration in seconds. """
        target, time, dt = yield None
        buffer = WeightBuffer()
        while True:
            buffer.append((target, dt))
            buffer.cut_to_fit(duration)
            target, time, dt = yield buffer.weighted_mean()

    @staticmethod
    def stick(duration, deviation=0.5):
        target, time, dt = yield None
        buffer = WeightBuffer()
        timerandom = lambda: min(max(random.gauss(0.35, 0.1), 0.2), 0.5)
        spacerandom = lambda sigma: [random.gauss(a, b) for a, b in
                                     zip(buffer.weighted_mean(), sigma)]
        nexttime = time
        while True:
            buffer.append((target, dt))
            buffer.cut_to_fit(duration)

            buffer_variance = buffer.weighted_variance()
            buffer_deviation = [sqrt(a) for a in buffer_variance]
            total_buffer_deviation = sqrt(sum(buffer_variance))

            if time >= nexttime:
                nexttime = timerandom() + time
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

    @staticmethod
    def chain(*transitions):
        for trans in transitions:
            trans.send(None)
        target, time, dt = yield None
        while True:
            value = target
            for trans in transitions:
                value = trans.send((value, time, dt))
            target, time, dt = yield value


class Wrappers:

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
    def wrap(transition, wrapper):
        prepend_transition, append_transition = wrapper
        return Transitions.chain(prepend_transition, transition, append_transition)


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

def fixed_fps(transition, fps):
    transition.send(None)
    target = yield None
    dt = 1.0/fps
    time = 0
    while True:
        target = yield transition.send((target, time, dt))
        time += dt


class Target:

    def __init__(self, base):
        self.base = _vectorize(base)
        self._accumulator = None

    @property
    def base(self):
        return _devectorize(self._base)

    @base.setter
    def base(self, val):
        """ A value that will not be cleared with the clear() method.
        Unlike add(), this method uses only the last value. """
        self._base = _vectorize(val)

    def add(self, val):
        """ Values that will be summed, added with 'base' and cleared on every
        call to clear(). """
        val = _vectorize(val)

        if self._accumulator == None:
            self._accumulator = val
        else:
            self._accumulator = [a + b for a, b in zip(self._accumulator, val)]

    def clear(self):
        maxlen = max(len(vec) if isinstance(vec, Sequence) else 0
                     for vec in [self._accumulator, self._base])

        # If any of the vectors are None, fill them with zeros.
        base = self._base or [0] * maxlen
        accumulator = self._accumulator or [0] * maxlen

        return [a + b for a, b in zip(accumulator, base)]


def _vectorize(value):
    if not isinstance(value, Sequence):
        return [value]
    else:
        return copy(value)

def _devectorize(vector):
    if len(vector) == 1:
        return vector[0]
    else:
        return copy(vector)
