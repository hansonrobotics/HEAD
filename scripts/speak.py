#!/usr/bin/env python

import os
import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String
import urllib

tts_cmd = (
  'wget -q -U "Mozilla/5.0" -O - '
  '"http://translate.google.com/translate_tts?tl=en-uk&q={}" > /tmp/speech.mp3'
)
sox_cmd = 'sox /tmp/speech.mp3 /tmp/speech.wav'

class ChatbotSpeaker:
  def __init__(self):
    rospy.init_node('chatbot_speaker')
    self._client = SoundClient()
    rospy.Subscriber('chatbot_responses', String, self._response_callback)
    rospy.spin()

  def _response_callback(self, data):
    query = urllib.quote(data.data)
    os.system(tts_cmd.format(query))
    os.system(sox_cmd)
    self._client.playWave('/tmp/speech.wav')

def main():
  speaker = ChatbotSpeaker()

if __name__ == '__main__':
  main()
