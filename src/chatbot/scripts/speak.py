#!/usr/bin/env python

import os
import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String
import urllib
import logging

logger = logging.getLogger('hr.chatbot.speak')

tts_cmd = (
  'simple_google_tts en '
  '"{}"'
)
sox_cmd = 'sox /tmp/speech.mp3 /tmp/speech.wav'

class ChatbotSpeaker:
  def __init__(self):
    rospy.init_node('chatbot_speaker')
    self._client = SoundClient()
    rospy.Subscriber('chatbot_responses', String, self._response_callback)
    rospy.spin()

  def _response_callback(self, data):
    os.system(tts_cmd.format(data.data))
    logger.warn('tts: %s',tts_cmd.format(data.data))
    #os.system(sox_cmd)
    #self._client.playWave('/tmp/speech.wav')

def main():
  speaker = ChatbotSpeaker()

if __name__ == '__main__':
  main()
