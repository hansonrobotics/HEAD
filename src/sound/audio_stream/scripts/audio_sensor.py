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
        self.freqs = deque(maxlen=5)
        self.rate = rospy.get_param('audio_rate', 16000)

        # https://en.wikipedia.org/wiki/Voice_frequency
        # The voiced speech of a typical adult male will have a fundamental
        # frequency from 85 to 180 Hz, and that of a typical adult female
        # from 165 to 255 Hz
        self.vad_lo_freq = 85
        self.vad_hi_freq = 255
        self.vad_decibel = 75

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
        THRESHOLD = 20
        if len(self.d) == self.d.maxlen:
            s = [self.d[i] for i in range(0, self.d.maxlen-1)]
            bg = sum(s)/(len(s)-1) # background noise level
            if self.d[-1]> bg+THRESHOLD:
                self.d.clear()
                return True
        return False

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

        avg_freq = sum(self.freqs)/len(self.freqs) if len(self.freqs) > 0 else 0
        if (self.vad_lo_freq < avg_freq < self.vad_hi_freq) and Decibel > self.vad_decibel:
            msg2.Speech = True
        self.pub.publish(msg2)

if __name__ == '__main__':
    rospy.init_node('audio_sensor')
    AudioSensor()
    while not rospy.is_shutdown():
        rospy.spin()
