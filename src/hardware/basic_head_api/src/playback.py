__author__ = 'vytas'
from threading import Timer
from time import time, sleep
from MotorCmder import MotorCmder

# plays animations
class Playback:
    def __init__(self, motors, pub):
        self._cmders = {}
        self._fps = 10
        self._motors = motors
        # publisher callback for motor command
        self._pub = pub

    # Starts animation while it finishes
    def _play(self, animation, fps):
        #start = time()
        dt = 1.0 / float(fps)
        t = time()
        for f in animation.frames():
            for m in f:
                if m in self._cmders:
                    msg = self._cmders[m].msg_fracDist(f[m])
                    self._pub(msg)
                else:
                    if m in self._motors:
                        self._cmders[m] = MotorCmder(self._motors[m])
                        msg = self._cmders[m].msg_fracDist(f[m])
                        self._pub(msg)

            # Exclude execution for more accurate timing
            t2 = time()
            i = dt - t2 + t
            if i > 0:
                sleep(i)
            t = time()

    def play(self, animation, fps):
        t = Timer(0.0, lambda: self._play(animation, fps))
        t.start()
