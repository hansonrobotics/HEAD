#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
import re
import logging
import os
import sys
import random
import threading
import pyglet
import subprocess

import SoundFile
from basic_head_api.msg import MakeFaceExpr
from std_msgs.msg import String
from tts.srv import TTSLengthResponse
from topic_tools.srv import MuxSelect
from blender_api_msgs.msg import Viseme
from visemes import BaseVisemes, English_Visemes

logger = logging.getLogger('hr.tts.tts_talker')

#CWD = os.path.dirname(os.path.realpath(__file__))
#sys.path.insert(0, os.path.join(CWD, '../pyfestival'))
#import festival

class TTSTalker:
    def __init__(self):
        topic = rospy.get_param('~topic_name', 'tts')
        rospy.Subscriber(topic, String, self.say)
        rospy.Subscriber(topic+'_en', String, self.say)
        self.speech_active = rospy.Publisher('speech_events', String, queue_size=10)
        self.vis_topic = rospy.Publisher('/blender_api/queue_viseme', Viseme, queue_size=0)
        self.lipsync_enabled = rospy.get_param('lipsync', True)
        self.lipsync_blender = rospy.get_param('lipsync_blender', True)
        self.mux = rospy.ServiceProxy('lips_pau_select', MuxSelect)
        self.expr_topic = rospy.Publisher('make_face_expr', MakeFaceExpr, queue_size=0)
        self.lipsync = False
        self.sound = SoundFile.SoundFile()
        self.output_dir = os.path.expanduser('~/.hr/tts/festival')
        if not os.path.isdir(self.output_dir):
            os.makedirs(self.output_dir)
        self.wavout = os.path.join(self.output_dir, 'tts.wav')
        self.timing = os.path.join(self.output_dir, 'timing')
        self.script = os.path.join(self.output_dir, 'tts.scm')
        self.viseme_config = English_Visemes()

    def tts_length(self, req):
        return TTSLengthResponse(1)

    def say(self, msg):
        text = msg.data
        self.startLipSync()
        self._say(text)
        self.stopLipSync()
        logger.info("Finished tts")

    def _say(self, text):
        try:
            logger.info('Say "{}"'.format(text))
            with open(self.script, 'w') as f:
                f.write("""
(voice_cmu_us_slt_arctic_hts)
(set! utt1 (Utterance Text "{text}"))
(utt.synth utt1)
(utt.save.segs utt1 "{timingfile}")
(utt.save.wave utt1 "{audiofile}")""".format(
                    text=text, timingfile=self.timing, audiofile=self.wavout)
                )
            subprocess.Popen(
                ['festival', '-b', self.script], stdout=subprocess.PIPE,
                stderr=subprocess.PIPE).communicate()
        except Exception as ex:
            import traceback
            logger.error('TTS error: {}'.format(traceback.format_exc()))
            return
        self.doLipSync()
        os.remove(self.wavout)
        os.remove(self.timing)
        os.remove(self.script)

    def startLipSync(self):
        self.lipsync = True
        self.speech_active.publish("start")
        if self.lipsync_enabled and not self.lipsync_blender:
            self.mux("lipsync_pau")

    def stopLipSync(self):
        self.lipsync = False
        self.speech_active.publish("stop")
        if self.lipsync_enabled and not self.lipsync_blender:
            self.mux("head_pau")

    def doLipSync(self):
        self.sound.start(self.wavout)

        phonemes = []
        with open(self.timing) as f:
            lines = f.read().splitlines()
            last_tick = 0
            for line in lines:
                if line.strip().startswith('#'):
                    continue
                tick, vol, phoneme  = line.strip().split(' ')
                tick = float(tick)
                phonemes.append({'name': phoneme, 'start': last_tick, 'end': tick})
                last_tick = tick
        visemes = self.viseme_config.get_visemes(phonemes)

        start = time.time()
        i = 0
        while i < len(visemes):
            if time.time() > start+visemes[i]['start']:
                logger.debug('{} viseme {}'.format(i, visemes[i]))
                self.sendVisime(visemes[i])
                i += 1
            time.sleep(0.001)
        self.sendVisime({'name': 'Sil'})

        elapsed_time = 0
        timeout = 10
        while self.sound.is_playing and elapsed_time < timeout:
            time.sleep(0.05)
            elapsed_time += 0.05
        logger.info('elapsed time {}'.format(elapsed_time))
        self.sound.stop()

    def sendVisime(self, visime):
        if self.lipsync_enabled and self.lipsync_blender and (visime['name'] != 'Sil'):
            #Need to have global shapekey_store class.
            msg = Viseme()
            # Duration should be overlapping
            msg.duration.nsecs = visime['duration']*1000000000*BaseVisemes.visemes_param[visime['name']]['duration']
            msg.name = visime['name']
            msg.magnitude = BaseVisemes.visemes_param[visime['name']]['magnitude']
            msg.rampin = BaseVisemes.visemes_param[visime['name']]['rampin']
            msg.rampout = BaseVisemes.visemes_param[visime['name']]['rampout']
            self.vis_topic.publish(msg)
        if self.lipsync_enabled and not self.lipsync_blender:
            msg = MakeFaceExpr()
            msg.exprname = 'vis_'+visime['name']
            msg.intensity = 1.0
            self.expr_topic.publish(msg)

if __name__ == '__main__':
    rospy.init_node('tts_talker')
    talker = TTSTalker()
    rospy.spin()
