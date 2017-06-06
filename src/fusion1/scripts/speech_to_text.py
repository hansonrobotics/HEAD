#!/usr/bin/env python2.7
import os
import rospy
import logging
import time
import dynamic_reconfigure.client
from fusion1.msg import Sound,Speech


# speech to text
class SpeechToText(object):


    def __init__(self):

        self.threshold = 0.2
        self.linger = 0.2

        rospy.wait_for_service("hearing_pipeline")
        self.dynparam = dynamic_reconfigure.client.Client("hearing_pipeline",timeout=30,config_callback=self.HandleConfig)        
        self.sound_sub = rospy.Subscriber("sound",Sound,self.HandleSound)
        self.speech_pub = rospy.Publisher("speech",Speech,queue_size=5)


    def HandleConfig(self,data,level):

        print "speech_to_text {}".format(data)
        return data

        
    def HandleSound(self,data):

        ()


if __name__ == '__main__':

    rospy.init_node('speech_to_text')
    node = SpeechToText()
    rospy.spin()
