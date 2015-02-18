#!/usr/bin/env python

import aiml
import rospy
import os
import sys

from chatbot.msg import ChatMessage
from std_msgs.msg import String

class Chatbot():
  def __init__(self):
    self._kernel = aiml.Kernel()
    rospy.init_node('chatbot_ai')
    rospy.Subscriber('chatbot_speech', ChatMessage, self._request_callback)
    self._response_publisher = rospy.Publisher(
      'chatbot_responses',
      String
    )

  def initialize(self, aiml_dir):
    self._kernel.learn(os.sep.join([aiml_dir, '*.aiml']))
    properties_file = open(rospy.get_param('~properties',os.sep.join([aiml_dir, 'bot.properties'])))
    for line in properties_file:
      parts = line.split('=')
      key = parts[0]
      value = parts[1]
      self._kernel.setBotPredicate(key, value)
    rospy.logwarn('Done initializing chatbot.')
    rospy.spin()

  def _request_callback(self, chat_message):
    response = ''
    if chat_message.confidence < 50:
      response = 'Could you say that again?'
    else:
      response = self._kernel.respond(chat_message.utterance)
    message = String()
    message.data = response
    self._response_publisher.publish(message)

def main():
  chatbot = Chatbot()
  aiml_dir = sys.argv[1]
  chatbot.initialize(aiml_dir)
    
if __name__ == '__main__':
  main()
