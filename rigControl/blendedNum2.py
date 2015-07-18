from collections import deque
from statistics import mean
import itertools
import math

class BlendedNum():
    ''' represents a number or vec3 float that is smoothly blended'''
    def __init__(self, value, transition=None):
        self._current = value
        self._target = Target()

        self.transition = transition or Transitions.moving_average(1.0)
        self.transition.send(None)


    @property
    def current(self):
        return self._current

    @property
    def target(self):
        return self._target

    def blend(self, time, dt):
        target_val = self._target.clear()
        self._current = self.transition.blend(target_val, time, dt)

class WeightBuffer(deque):

    def weighted_mean(self):
        result = []

        vectors, weights = zip(*self)
        for dimension in zip(*vectors):
            single_dimension_result = WeightBuffer(zip(dimension, weights))._weighted_mean_scalar()
            result.append(single_dimension_result)

        return result

    def _weighted_mean_scalar(self):
        weighted_sum = sum(val * weight for val, weight in self)
        sum_of_weights = sum(weight for val, weight in self)
        return weighted_sum / sum_of_weights

    def cut_to_fit(self, sum_of_weights):
        """ Trims the left side until the buffer matches the given 'sum_of_weights'.
        E.g. WeightBuffer([(..., 0.2), (..., 0.8), (..., 0.4)]).cut_to_fit(1.0)
        would result in WeightBuffer([(..., 0.6), (..., 0.4)])
        """
        overflow = sum(weight for val, weight in self) - sum_of_weights

        if overflow <= 0:
            return

        while overflow > 0:
            val, weight = self.popleft()
            overflow -= weight

        if overflow < 0:
            self.appendleft((val, abs(overflow)))

def fixed_fps(transition, fps):
    transition.send(None)
    target = yield None
    dt = 1.0/fps
    time = 0
    while True:
        target = yield transition.send((target, time, dt))
        time += dt


class Transitions:

    @staticmethod
    def linear(speed):
        # 'target' type: List[int]
        target, time, dt = yield None
        current = target
        while True:
            # Find the distance to 'target'
            displacement = [a - b for a, b in zip(target, current)]
            distance = math.sqrt(sum(a**2 for a in displacement))

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
        # target type: List[int]
        target, time, dt = yield None
        buffer = WeightBuffer()
        while True:
            buffer.append((target, dt))
            buffer.cut_to_fit(duration)
            target, time, dt = yield buffer.weighted_mean()

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


def result1():
    gen = Transitions.moving_average(1)
    gen.send(None)
    lst = [0] * 10 + list(range(10)) + [10] * 10
    lst = list(zip(lst, itertools.repeat(0), itertools.repeat(0.2)))
    return [gen.send(el) for el in lst]

def result2():
    gen = Transitions.moving_average(1)
    gen.send(None)
    lst = list(zip([0] * 10, itertools.repeat(0), itertools.repeat(0.2)))
    lst.extend(zip([i/2 for i in range(20)], itertools.repeat(0), itertools.repeat(0.1)))
    lst.extend(zip([10] * 10, itertools.repeat(0), itertools.repeat(0.2)))
    return [gen.send(el) for el in lst]

def mvavg(lst, n):
    buffer = deque(maxlen=n)
    result = []
    for el in lst:
        buffer.append(el)
        result.append(mean(buffer))
    return result

def der(lst):
    return [0] + [x - xprev for x, xprev in zip(lst[1:], lst[:-1]) ]


class Target:

    def __init__(self):
        self._accumulator = 0

    def add(self, val):
        self._accumulator += val

    def clear(self):
        value = self._accumulator
        self._accumulator = 0
        return value
