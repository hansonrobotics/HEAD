# Implements a numeric object that allow blending and smoothing of values
# access using .current, set using .target, must be manually interpolated using .blend() every frame

import collections

class BlendedNum():
    ''' represents a number or vec3 float that is smoothly blended'''
    def __init__(self, value, steps = 20, smoothing = 20):
        if type(value) is float or type(value) is int:
            self._target = value
            self._old = value
            self._current = value
            self._movingAverage = collections.deque(5*[0], smoothing)
            self.isVector = False
        else:
            self._target = value.copy()
            self._old = value.copy()
            self._current = value.copy()
            self._movingAverage = collections.deque(5*[value.copy()], smoothing)
            self.isVector = True

        self._steps = steps


    def __repr__(self):
        allStr = 'BlendedNum:' + str((self._current, self._old, self._target))
        return allStr

    @property
    def current(self):
        ''' return moving average instead of raw lerp value '''
        if self.isVector:
            return [float(sum(col))/len(col) for col in zip(*list(self._movingAverage))]
        else:
            return sum(self._movingAverage)/len(self._movingAverage)

    @current.setter
    def current(self, value):
        self._current = value


    @property
    def target(self):
        return self._target

    @target.setter
    def target(self, value):
        if self.isVector:
            self._target = value.copy()
            self._old = self._current.copy()
        else:
            self._target = value
            self._old = self._current


    @property
    def steps(self):
        return self._steps
    @steps.setter
    def steps(self, value):
        self._steps = value

    # @property
    # def smoothing(self):
    #     return self._movingAverage
    # @smoothing.setter
    # def smoothing(self, smoothing):
    #     self._movingAverage = collections.deque(5*[0], smoothing)

    def blend(self):
        #store old value to moving average:
        if self.isVector:
            self._movingAverage.append(self._current.copy())
        else:
            self._movingAverage.append(self._current)

        if self.isVector and len(self._target)==3:
            # trilinear interpolation
            delta = [0,0,0]
            delta[0] = (self._target[0] - self._old[0]) / self._steps
            delta[1] = (self._target[1] - self._old[1]) / self._steps
            delta[2] = (self._target[2] - self._old[2]) / self._steps

            self._current[0] += delta[0]
            self._current[1] += delta[1]
            self._current[2] += delta[2]

            if (delta[0] > 0 and self._current[0] > self._target[0]) or (delta[0] < 0 and self._current[0] < self._target[0]):
                self._current[0] = self._target[0]
                self._old[0] = self._target[0]
            if (delta[1] > 0 and self._current[1] > self._target[1]) or (delta[1] < 0 and self._current[1] < self._target[1]):
                self._current[1] = self._target[1]
                self._old[1] = self._target[1]
            if (delta[2] > 0 and self._current[2] > self._target[2]) or (delta[2] < 0 and self._current[2] < self._target[2]):
                self._current[2] = self._target[2]
                self._old[2] = self._target[2]

        else:
            # linear interpolation
            deltaValue = (self._target - self._old) / self._steps
            self._current += deltaValue

            if (deltaValue > 0 and self._current > self._target) or (deltaValue < 0 and self._current < self._target):
                self._current = self._target
                self._old = self._target
                return
