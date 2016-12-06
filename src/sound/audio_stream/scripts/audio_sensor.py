#!/usr/bin/env python

import rospy
import math
import struct
import numpy as np
from collections import deque
from std_msgs.msg import Float32, UInt8MultiArray
from audio_stream.msg import audiodata
from audio_stream.frequency_estimator import freq_from_fft

class AudioSensor(object):

    def __init__(self):
        self.pub = rospy.Publisher(
            'audio_sensors', audiodata, queue_size=1)
        rospy.Subscriber('speech_audio', UInt8MultiArray, self.audio_cb)
        self.d = deque(maxlen=5)
        self.freqs = deque(maxlen=10)
        self.rate = rospy.get_param('audio_rate', 16000)

    def convData(self, V):
        # Converts Stream (which is in byte format) to List of +ve and -ve
        # Integers
        count = len(V) / 2
        format = "%dh" % (count)
        shorts = struct.unpack(format, V)
        return shorts

    def get_decibel(self, block):
        # The Energy of Sound per chunk is calculated
        a = np.asarray(block)
        rms = np.sqrt(np.mean(np.absolute(a)**2))
        if rms > 1:
            p = 20 * math.log10(rms)
        else:
            p = 0
        return p

    def suddenChanges(self):
        # Return either 1 or 0 for Sudden Change and Similar decibels
        # respectively
        THRESHOLD = 25  # Test and modification of this constant is required
        if max(self.d) - min(self.d) > THRESHOLD:
            return 1
        else:
            return 0

    def audio_cb(self, msg):
        AudioData = self.convData(msg.data)
        Decibel = self.get_decibel(AudioData)
        freq = freq_from_fft(AudioData, self.rate)
        self.d.append(Decibel)
        self.freqs.append(freq)

        msg2 = audiodata()
        msg2.Decibel = Decibel
        msg2.Frequency = freq
        msg2.SuddenChange = self.suddenChanges()

        # https://en.wikipedia.org/wiki/Voice_frequency
        # The voiced speech of a typical adult male will have a fundamental
        # frequency from 85 to 180 Hz, and that of a typical adult female
        # from 165 to 255 Hz
        avg_freq = sum(self.freqs)/len(self.freqs) if len(self.freqs) > 0 else 0
        if 85 < avg_freq < 255 and Decibel > 70:
            msg2.speech = True
        self.pub.publish(msg2)

if __name__ == '__main__':
    rospy.init_node('audio_sensor')
    AudioSensor()
    while not rospy.is_shutdown():
        rospy.spin()
