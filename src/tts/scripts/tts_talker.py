#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
import logging
import os
import random

from std_msgs.msg import String
from blender_api_msgs.msg import Viseme
from common.visemes import BaseVisemes
from tts.ttsapi import get_api
from tts import SoundFile
from tts.srv import TTSLengthResponse

logger = logging.getLogger('hr.tts.tts_talker')

class TTSTalker:
    def __init__(self):
        topic = rospy.get_param('~topic_name', 'chatbot_responses')
        tts_control = rospy.get_param('tts_control', 'tts_control')
        rospy.Subscriber(topic, String, self.say)
        rospy.Subscriber(topic+'_en', String, self.say, 'en')
        self.speech_active = rospy.Publisher('speech_events', String, queue_size=10)
        self.vis_topic = rospy.Publisher('/blender_api/queue_viseme', Viseme, queue_size=0)
        self.blink_publisher = rospy.Publisher('chatbot_blink', String, queue_size=1)
        self.sound = SoundFile.SoundFile()
        self.tts_data = None
        self.interrupt = False
        rospy.Subscriber(tts_control, String, self.tts_control)

    def _get_tts_api_config(self):
        return rospy.get_param('tts_api_config', {'en': 'festival'})

    def tts_length(self, req):
        return TTSLengthResponse(1)

    def tts_control(self, msg):
        if msg.data == 'shutup':
            logger.info("Shut up!!")
            self.sound.interrupt()
            self.interrupt = True

    def say(self, msg, lang=None):
        if lang is None:
            lang = rospy.get_param('lang', None)
            if lang is None:
                logger.error("Language is not set")
                return
        text = msg.data
        self.interrupt = False
        self.startLipSync()
        self._say(text, lang)
        self.stopLipSync()
        logger.info("Finished tts")

    def _say(self, text, lang):
        logger.info('Say "{}" in {}'.format(text, lang))
        self.tts_data = None
        tts_api_config = self._get_tts_api_config()
        api_name = tts_api_config.get(lang, None)
        api = get_api(api_name)
        if api is None:
            logger.error("No TTS API")
            return
        self.tts_data = api.tts(text)
        if self.tts_data and hasattr(self.tts_data, "visemes"):
            self.doLipSync()

    def startLipSync(self):
        self.speech_active.publish("start")

    def stopLipSync(self):
        self.speech_active.publish("stop")

    def doLipSync(self):
        self.sound.start(self.tts_data.wavout)

        visemes = self.tts_data.visemes
        start = time.time()
        i = 0
        while i < len(visemes) and not self.interrupt:
            if time.time() > start+visemes[i]['start']:
                logger.debug('{} viseme {}'.format(i, visemes[i]))
                self.sendVisime(visemes[i])
                # blink at next to last syllable with probability .7
                if i == (len(visemes)-2):
                    if random.random() < 0.7:
                        self.blink_publisher.publish(String('tts_end'))
                i += 1
            time.sleep(0.001)
        self.sendVisime({'name': 'Sil'})

        elapsed_time = 0
        timeout = 0.5
        while self.sound.is_playing and elapsed_time < timeout and not self.interrupt:
            time.sleep(0.05)
            elapsed_time += 0.05
        logger.info('elapsed time {}'.format(elapsed_time))
        self.sound.stop()

    def sendVisime(self, visime):
        if visime['name'] != 'Sil':
            msg = Viseme()
            msg.duration.nsecs = visime['duration']*1000000000*BaseVisemes.visemes_param[visime['name']]['duration']
            msg.name = visime['name']
            msg.magnitude = BaseVisemes.visemes_param[visime['name']]['magnitude']
            msg.rampin = BaseVisemes.visemes_param[visime['name']]['rampin']
            msg.rampout = BaseVisemes.visemes_param[visime['name']]['rampout']
            self.vis_topic.publish(msg)
        else:
            # Send silence viseme: Using M instead
            msg = Viseme()
            msg.duration.nsecs = 100000000
            msg.name = 'M'
            self.vis_topic.publish(msg)

if __name__ == '__main__':
    rospy.init_node('tts_talker')
    talker = TTSTalker()
    rospy.spin()
