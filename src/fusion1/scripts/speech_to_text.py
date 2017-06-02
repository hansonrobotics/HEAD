#!/usr/bin/env python2.7
import os
import rospy
import logging
import time
from fusion1.msg import Sound,Speech


# speech to text
class SpeechToText(object):


    def __init__(self):

        self.threshold = 0.2
        self.linger = 0.2

        self.sound_sub = rospy.Subscriber("sound",Sound,self.HandleSound)
        self.speech_pub = rospy.Publisher("speech",Speech,queue_size=5)


    def HandleSound(self,data):

        ()


if __name__ == '__main__':

    rospy.init_node('speech_to_text')
    node = SpeechToText()
    rospy.spin()
