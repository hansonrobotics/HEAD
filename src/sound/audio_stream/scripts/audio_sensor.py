#!/usr/bin/env python

import rospy
import math
import struct
import numpy as np
from collections import deque
from std_msgs.msg import Float32, UInt8MultiArray
from audio_stream.msg import audiodata

class AudioSensor(object):

    def __init__(self):
        self.audio_feature = rospy.Publisher(
            '/opencog/AudioFeature', audiodata, queue_size=1)
        rospy.Subscriber('speech_audio', UInt8MultiArray, self.audio_cb)
        self.d = deque(maxlen=5)

    def convData(self, V):
        # Converts Stream (which is in byte format) to List of +ve and -ve
        # Integers
        count = len(V) / 2
        format = "%dh" % (count)
        shorts = struct.unpack(format, V)
        return shorts

    def get_decibel(self, block):
        # The Energy of Sound per chunk is calculated
        # get 1 val for each two char(byte)
        count = len(block) / 2
        SQUARE = 0.0
        for i in block:
            SQUARE = SQUARE + math.pow(abs(i), 2)
        p = 20 * math.log10(math.sqrt((SQUARE / count)))
        return p

    def getFreq(self, Maindata):
        rate = rospy.get_param('audio_rate', 16000)
        chunk = int(rate / 10)  # 100ms

        # Get the frequency of the current Chunk
        # Purpose: Get the current Pitch of the file using fast fourier
        # transform
        fftData = abs(np.fft.rfft(Maindata)) ** 2  # find the maximum
        which = fftData[1:].argmax() + 1
        # use quadratic interpolation around the max
        if which != len(fftData) - 1:
            y0, y1, y2 = np.log(fftData[which - 1:which + 2:])
            x1 = (y2 - y0) * .5 / (2 * y1 - y2 - y0)
            # find the frequency and output it
            FREQUENCY = (which + x1) * rate / chunk
        else:
            FREQUENCY = which * rate / chunk
        return FREQUENCY

    def suddenChanges(self):
        # Return either 1 or 0 for Sudden Change and Similar decibels
        # respectively
        THRESHOLD = 25  # Test and modification of this constant is required
        if max(self.d) - min(self.d) > THRESHOLD:
            return 10
        else:
            return 0

    def audio_cb(self, msg):
        AudioData = self.convData(msg.data)
        Decibel = self.get_decibel(AudioData)
        self.d.append(Decibel)

        msg2 = audiodata()
        msg2.Decibel = Decibel
        msg2.Frequency = self.getFreq(AudioData)
        msg2.SuddenChange = self.suddenChanges()
        self.audio_feature.publish(msg2)

if __name__ == '__main__':
    rospy.init_node('audio_sensor')
    AudioSensor()
    while not rospy.is_shutdown():
        rospy.spin()
