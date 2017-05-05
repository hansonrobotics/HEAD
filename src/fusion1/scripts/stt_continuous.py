#!/usr/bin/env python2.7
import os
import rospy
import logging

from fusion1.msg import ...
# dynamic reconfigure server?

logger = logging.getLogger('hr.fusion1')


class STTContinuous(object):

    def __init__(self):
    	self.audio_sub = rospy.Subscriber("audio_feed",Audio,self.HandleAudio)
    	self.speech_pub = rospy.Publisher("speech",Speech,queue_size=1)

    def HandleAudio(self,data):
        # get current timestamp
    	# send to continuous STT, track timestamp
    	# ...from whatever comes back, publish speech
    	

if __name__ == '__main__':
    rospy.init_node('stt_continuous')
    node = STTContinuous()
    rospy.spin()
