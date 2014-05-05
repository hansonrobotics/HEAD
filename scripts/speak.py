#!/usr/bin/env python

import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String

class ChatbotSpeaker:
  def __init__(self):
    rospy.init_node('chatbot_speaker')
    self._client = SoundClient()
    rospy.Subscriber('chatbot_responses', String, self._response_callback)
    rospy.spin()

  def _response_callback(self, data):
    self._client.say(data.data)

def main():
  speaker = ChatbotSpeaker()

if __name__ == '__main__':
  main()
