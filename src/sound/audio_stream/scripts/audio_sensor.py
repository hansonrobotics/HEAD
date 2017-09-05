#!/usr/bin/env python

import os
import rospy
import math
import struct
import numpy as np
from collections import deque
import wave
import contextlib
import logging

from std_msgs.msg import Float32, UInt8MultiArray, String
from audio_stream.msg import audiodata
from audio_stream.frequency_estimator import freq_from_fft

if os.path.isdir('/opt/hansonrobotics/lib/python2.7/site-packages'):
    import sys
    sys.path.insert(0, '/opt/hansonrobotics/lib/python2.7/site-packages')

from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *

import webrtcvad

MODELDIR = "/opt/hansonrobotics/share/pocketsphinx/model"

logger = logging.getLogger('hr.audio_stream.audio_sensor')

class AudioSensor(object):

    def __init__(self):
        self.decocer_config = Decoder.default_config()
        self.decocer_config.set_string(
            '-hmm', os.path.join(MODELDIR, 'en-us/en-us'))
        self.decocer_config.set_string(
            '-lm', os.path.join(MODELDIR, 'en-us/en-us.lm.bin'))
        self.decocer_config.set_string(
            '-dict', os.path.join(MODELDIR, 'en-us/cmudict-en-us.dict'))
        self.decoder = Decoder(self.decocer_config)
        self.vad = webrtcvad.Vad()

        self.pub = rospy.Publisher(
            'audio_sensors', audiodata, queue_size=1)
        self.chat_event_pub = rospy.Publisher(
            'chat_events', String, queue_size=1)
        self.d = deque(maxlen=5)
        self.freqs = deque(maxlen=5)
        self.rate = rospy.get_param('audio_rate', 16000)
        self.speech = False
        self.speech_buf = deque(maxlen=5)
        self.chat_start = False

        # https://en.wikipedia.org/wiki/Voice_frequency
        # The voiced speech of a typical adult male will have a fundamental
        # frequency from 85 to 180 Hz, and that of a typical adult female
        # from 165 to 255 Hz
        self.vad_lo_freq = 85
        self.vad_hi_freq = 255
        self.vad_decibel = 75
        rospy.Subscriber('speech_audio', UInt8MultiArray, self.audio_cb)

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

    def vad_change_event(self):
        if not self.speech:
            self.speech_buf.clear()
            if self.chat_start:
                self.chat_event_pub.publish('speechstop')
                self.chat_start = False

    def get_text(self):
        try:
            if self.speech_buf:
                self.decoder.start_utt()
                for buf in self.speech_buf:
                    self.decoder.process_raw(buf, False, False)
                self.decoder.end_utt()
                self.speech_buf.clear()
                text = [seg.word for seg in self.decoder.seg()]
                self.current_speech_buf_len = 0
                if text and len(text) > 2:
                    return ' '.join(text[1:-1])
        except Exception as ex:
            logger.error(ex)

    def audio_cb(self, msg):
        AudioData = self.convData(msg.data)
        Decibel = self.get_decibel(AudioData)
        freq = freq_from_fft(AudioData, self.rate)
        self.d.append(Decibel)
        self.freqs.append(freq)

        frame = msg.data[:960] # 30ms
        speech = self.vad.is_speech(frame, self.rate)
        if self.speech != speech:
            self.speech = speech
            self.vad_change_event()
        if self.speech:
            self.speech_buf.append(msg.data)
        if not self.speech or len(self.speech_buf) == self.speech_buf.maxlen:
            text = self.get_text()
            if text:
                logger.info('Best hypothesis segments: {}'.format(text))
                if not self.chat_start:
                    self.chat_event_pub.publish('speechstart:{}'.format(text))
                    self.chat_start = True

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
