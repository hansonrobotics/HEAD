__author__ = 'vytas'
from threading import Timer, Lock, ThreadError
from time import time, sleep
from MotorCmder import MotorCmder
import rospy

# plays animations
class Playback:
    def __init__(self, motors, pub, channels):
        self._cmders = {}
        self._fps = 10
        self._motors = motors
        # publisher callback for motor command
        self._pub = pub
        # channels
        self._channels = {}
        for c in channels:
            self._channels[c] = Lock()
        self._last_postions = {}



    # Starts animation while it finishes
    def _play(self, animation, fps, name):
        if not self._acquireChannel(name):
            rospy.logerr("Unable to play %s. Anopther animation is running.")
            return
        try:
            #start = time()
            dt = 1.0 / float(fps)
            t = time()
            for f in animation.frames(self._last_postions):
                for m in f:
                    if m in self._cmders:
                        msg = self._cmders[m].msg_fracDist(f[m])
                        self._pub(msg)
                    else:
                        if m in self._motors:
                            self._cmders[m] = MotorCmder(self._motors[m])
                            msg = self._cmders[m].msg_fracDist(f[m])
                            self._pub(msg)
                    self._last_postions[m] = f[m]

                # Exclude execution for more accurate timing
                t2 = time()
                i = dt - t2 + t
                if i > 0:
                    sleep(i)
                t = time()
        except:
            rospy.logerr("Error play animation {}".format(animation))
        finally:
            self._releaseChannel(name)

    def _acquireChannel(self, name):
        for k in self._channels.keys():
            if name[:len(k)] == k:
                return self._channels[k].acquire(False)
        return True

    def _releaseChannel(self, name):
        for k in self._channels.keys():
            if name[:len(k)] == k:
                try:
                    self._channels[k].release()
                except ThreadError:
                    pass
                return
        return


    def play(self, animation, fps, name):
        t = Timer(0.0, lambda: self._play(animation, fps, name))
        t.start()


