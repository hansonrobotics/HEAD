#!/usr/bin/env python2.7
import os
import rospy
import numpy
import time
import cv2

from dynamic_reconfigure.server import Server
from fusion1.cfg import HearingConfig
from fusion1.msg import Sound,Speech


sounds_base_dir = os.getenv("HOME") + "/hansonrobotics/HEAD/src/fusion1/test/sounds"


class HearingPipeline(object):


    def __init__(self):

        self.name = os.path.basename(rospy.get_namespace())
        self.session_tag = rospy.get_param("/session_tag")
        self.session_id = hash(session_tag) & 0xFFFFFFFF

        self.microphone_id = hash(name) & 0xFFFFFFFF

        today_tag = time.strftime("%Y%m%d")
        microphone_tag = self.name + "_%08X" % (self.microphone_id & 0xFFFFFFFF)
        self.sounds_dir = sounds_base_dir + "/" + today_tag + "/" + session_tag + "_%08X/" % (self.session_id & 0xFFFFFFFF) + microphone_tag + "/"
        if not os.path.exists(self.sounds_dir):
            os.makedirs(self.sounds_dir)

        self.pusers = {}

        self.sound_sub = rospy.Subscriber(self.name + "/sound",Sound,self.HandleSound)
        self.speech_sub = rospy.Subscriber(self.name + "/speech",Speech,self.HandleSpeech)

        self.hearing_pub = rospy.Publisher("hearing",Hearing,queue_size=5)

        self.config_srv = Server(HearingConfig,HandleConfig)


    def HandleSound(self,data):

        sound_tag = "sound_%08X" % (data.sound_id & 0xFFFFFFFF)
        sound_file = dir + face_tag + ".wav"
        # write sound


    def HandleSpeech(self,data):

    	() # write speech in CSV and transmit to fusion


if __name__ == '__main__':

    rospy.init_node('hearing_pipeline')
    node = HearingPipeline()
    rospy.spin()
