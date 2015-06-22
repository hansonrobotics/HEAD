#!/usr/bin/env python

import aiml
import rospy
import os
import sys

from chatbot.msg import ChatMessage
from std_msgs.msg import String
from blender_api_msgs.msg import EmotionState

class Chatbot():
  def __init__(self):
    self._kernel = aiml.Kernel()
    rospy.init_node('chatbot_ai')
    rospy.Subscriber('chatbot_speech', ChatMessage, self._request_callback)
    self._response_publisher = rospy.Publisher(
      'chatbot_responses', String, queue_size=1
    )

    # Perceived emotional content; and emotion to express
    # Perceived: based on what chatbot heard, this is how robot should
    # feel.  Expressed: the emotional content that the chatbot should
    # put into what it says.
    self._affect_publisher = rospy.Publisher(
      'chatbot_affect_perceive',
      EmotionState, queue_size=1
    )
    rospy.Subscriber('chatbot_affect_express', EmotionState,
        self._affect_express_callback)

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

  # Tell the world the emotion that the chatbot is perceiving.
  # Use the blender_api_msgs/EmotionState messae type to
  # describe the perceived emotion.  Argument is just a string.
  def _affect_perceive(self, emo):
    rospy.logwarn("Chatbot perceived emo:", emo)
    exp = EmotionState()
    exp.name = emo
    exp.magnitude = 1.0
    exp.duration.secs = 0
    exp.duration.nsecs = 0
    self._affect_publisher.publish(exp)

  # This is the emotion that the chatbot should convey.
  # affect_message is of type blender_api_msgs/EmotionState
  # Fields are name (String) magnitude (float), duration (time)
  def _affect_express_callback(self, affect_message):
    #
    rospy.logwarn("Chatbot is verbally expressing this: " +
       affect_message.name)

def main():
  chatbot = Chatbot()
  aiml_dir = sys.argv[1]
  chatbot.initialize(aiml_dir)

if __name__ == '__main__':
  main()
