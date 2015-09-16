from time import time as systime
from .plumbing import Pipes, Sources, Plumbing
from .plumbing import _vectorize, _devectorize, _flatten

__doc__ = """
A collections of 'Blended' numbers, numbers that change over time according to
specified rules.

All classes here should implement the method 'blend(time, dt)' and property
'current' so they'd be interchangable from the playback's point of view.
"""

class LiveTarget():

    """
    A number that chases a target in a fashion specified by pipes (the
    transition property). The target is free to be modified on every frame.

    The function of this class closely resembles the original version of BlendedNum,
    when the whole blendedNum package consisted only of one class.
    """

    def __init__(self, value, transition=None, target=None):
        value = _vectorize(value)
        self._current = value

        self.plumbing = Plumbing()
        self.plumbing.source = Sources.AdditiveTarget(target if target is not None else value)
        self.transition = transition or Pipes.identity()

    @property
    def current(self):
        return _devectorize(self._current)

    @property
    def target(self):
        return self.plumbing.source.base

    @target.setter
    def target(self, val):
        """ Set the target to move towards when 'blend' is called. """
        self.plumbing.source.base = val

    def target_add(self, val):
        """ Add alterations to the base target value. These alterations are
        cleared on every 'blend'. """
        self.plumbing.source.add(val)

    def blend(self, time, dt):
        """ Updates the 'current' value and clears any modifications added to
        the target with target_add(val). """
        self._current = self.plumbing.blend(time, dt)
        return self.current

    @property
    def transition(self):
        return self.plumbing.pipes

    @transition.setter
    def transition(self, transition):
        pipes = _vectorize(transition)
        if pipes == self.plumbing.pipes:
            return

        # Make the pipes think they've been at the current position for 10
        # seconds for initialization purposes.
        Plumbing(source=Sources.constant(self._current), pipes=pipes).blend(systime(), 10.0)

        self.plumbing.pipes = pipes


class Trajectory:

    """
    A number that moves in a pre-defined way specified by keyframes.
    Keyframes can be added before or after the trajectory started playing.

    See README.md in blendedNum package for some examples on how to use this class.
    """

    def __init__(self, value):
        self._current = _vectorize(value)
        self.time = 0
        self._transitions = {}
        self.keyframes = []
        self.plumbing = None

    def add_keyframe(self, target, time=None, transition=None):
        self.keyframes.append(self.Keyframe(target, time, transition))
        if len(self.keyframes) == 1:
            self._absorb_keyframe(self.keyframes[0])
        return self

    def _absorb_keyframe(self, keyframe):
        # Make the new pipes think they've been at the current position for 10
        # seconds for initialization purposes.
        for index, pipe in keyframe.transitions:
            Plumbing(source=Sources.constant(self._current),
                     pipes=_vectorize(pipe)).blend(self.time, 10.0)

        self._transitions.update(keyframe.transitions)
        self.plumbing = Plumbing(source=Sources.constant(keyframe.target),
                                 pipes=_flatten([b for a, b, in sorted(self._transitions.items())]))

    def set_transition(self, index, pipe):
        self._transitions[index] = pipe
        self.plumbing.pipes = _flatten([b for a, b, in sorted(self._transitions.items())])

    @property
    def current(self):
        return _devectorize(self._current)

    def blend(self, time, dt):
        self.time += dt

        # If next keyframe is due, update the plumbing
        if len(self.keyframes) > 0:
            kf = self.keyframes[0]
            if (kf.time != None and self.time >= kf.time or
                kf.time == None and sum((a-b)**2 for a,b in zip(self._current, self.keyframes[0].target)) < 0.001):
                self.keyframes = self.keyframes[1:]
                if len(self.keyframes) > 0:
                    self._absorb_keyframe(self.keyframes[0])

        self._current = self.plumbing.blend(time, dt)
        return self._current

    @property
    def is_done(self):
        return len(self.keyframes) == 0

    class Keyframe:

        def __init__(self, target, time=None, transitions=None):
            self.target = _vectorize(target)
            self.time = time
            if transitions == None:
                self.transitions = []
            elif isinstance(transitions, tuple):
                self.transitions = [transitions]
            else:
                self.transitions = transitions

        def __repr__(self):
            props = ['target={}'.format(self.target)]
            if self.time != None:
                props.append('time={}'.format(self.time))
            if self.transitions:
                props.append('transitions={}'.format(self.transitions))
            return '<Keyframe {}>'.format(' '.join(props))

    def __repr__(self):
        return '<blendedNum.Trajectory current={}\n transitions={}\n keyframes={}>'.format(
            self._current, self._transitions, self.keyframes)
