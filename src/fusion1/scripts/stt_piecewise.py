#!/usr/bin/env python2.7
import os
import rospy
import logging

from fusion1.msg import AudioChunk,Speech
# dynamic reconfigure server?

logger = logging.getLogger('hr.fusion1')


class STTPiecewise(object):

    def __init__(self):
    	self.audio_sub = rospy.Subscriber("audio_chunk",AudioChunk,self.HandleAudioChunk)
    	self.speech_pub = rospy.Publisher("speech",Speech,queue_size=1)

    def HandleAudioChunk(self,data):
    	# get current timestamp
    	# execute STT on audio chunk
    	# if anything came back, publish speech


if __name__ == '__main__':
    rospy.init_node('stt_piecewise')
    node = STTPiecewise()
    rospy.spin()
