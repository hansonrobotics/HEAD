#!/usr/bin/env python2.7
import os
import rospy
import logging

from fusion1.msg import ...
# dynamic reconfigure server?

logger = logging.getLogger('hr.fusion1')


class DetectSpeech(object):

    def __init__(self):
    	self.audio_sub = rospy.Subscriber("audio",Audio,self.HandleAudio)
    	self.audio_chunk_pub = rospy.Publisher("audio_chunk",AudioChunk,queue_size=1)

    def HandleAudio(self,data):
    	# analyze samples, detect if there is speech, if so, "start recording"
    	# when speech is done, publish recording as audio chunk


if __name__ == '__main__':
    rospy.init_node('detect_speech')
    node = DetectSpeech()
    rospy.spin()
