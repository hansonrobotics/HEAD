#!/usr/bin/env python

import rospy
import os
import logging
import argparse
import requests
import json

from chatbot.msg import ChatMessage
from std_msgs.msg import String
from polarity import Polarity

logger = logging.getLogger('hr.chatbot.ai')

class Chatbot():
  def __init__(self, botname):
    self.botname = botname
    self.chatbot_url = 'http://localhost:8001'

    # chatbot now saves a bit of simple state to handle sentiment analysis
    # after formulating a response it saves it in a buffer if S.A. active
    # It has a simple state transition - initialized in wait_client
    # after getting client if S.A. active go to wait_emo
    #  in affect_express call back publish response and reset to wait_client
    self._response_buffer = ''
    self._state = 'wait_client'
    # argumment must be  to activate sentiment analysis
    self._sentiment_active=False
    # sentiment dictionary
    self.polarity = Polarity()
    self._polarity_threshold=0.2

    rospy.Subscriber('chatbot_speech', ChatMessage, self._request_callback)

    self._response_publisher = rospy.Publisher(
        'chatbot_responses', String, queue_size=1)

    # send communication non-verbal blink message to behavior
    self._blink_publisher = rospy.Publisher(
        'chatbot_blink',String,queue_size=1)

    # Perceived emotional content; and emotion to express
    # Perceived: based on what chatbot heard, this is how robot should
    # feel.  Expressed: the emotional content that the chatbot should
    # put into what it says.
    self._affect_publisher = rospy.Publisher(
        'chatbot_affect_perceive', String, queue_size=1)

    # Echo chat messages as plain strings.
    self._echo_publisher = rospy.Publisher('perceived_text', String, queue_size=1)
    rospy.Subscriber('chatbot_speech', ChatMessage, self._echo_callback)

  def sentiment_active(self, active):
    self._sentiment_active = active

  def get_response(self, question):
      r = requests.post(self.chatbot_url,
            data = json.dumps(
                {"botname":"{}".format(self.botname),
                "question":"{}".format(question),
                "session":"0"}),
            headers = {"Content-Type": "application/json"}
      )
      ret = r.json().get('ret')
      if r.status_code != 200:
        logger.error("Request error: {}".format(r.status_code))

      if ret != 0:
        logger.error("QA error: {}".format(ret))

      response = r.json().get('answer')

      if r.status_code != 200 or ret != 0 or not response:
        response = question

      return response

  def _request_callback(self, chat_message):
    if rospy.get_param('lang', None) != 'en':
        logger.info('Ignore non-English language')
        return
    response = ''

    blink=String()
    # blink that we heard something, request, probability defined in callback
    blink.data='chat_heard'
    self._blink_publisher.publish(blink)

    if chat_message.confidence < 50:
      response = 'Could you say that again?'
      message = String()
      message.data = response
      self._response_publisher.publish(message)
      # puzzled expression
    else:
      # request blink, probability of blink defined in callback
      blink.data='chat_saying'
      self._blink_publisher.publish(blink)

      response = self.get_response(chat_message.utterance)

      # Add space after punctuation for multi-sentence responses
      response = response.replace('?','? ')
      response = response.replace('.','. ')
      response = response.replace('_',' ')

      # if sentiment active save state and wait for affect_express to publish response
      # otherwise publish and let tts handle it
      if self._sentiment_active:
        p = self.polarity.get_polarity(response)
        logger.info('Polarity for "{}" is {}'.format(response, p))
        emo = String()
        # change emotion if polarity magnitude exceeds threshold defined in constructor
        # otherwise let top level behaviors control
        if p > self._polarity_threshold:
          emo.data = 'happy'
          self._affect_publisher.publish(emo)
          logger.info('Chatbot perceived emo: {}'.format(emo.data))
          # Currently response is independant of message received so no need to wait
          # Leave it for Opencog to handle responses later on.
        elif p < 0 and abs(p)> self._polarity_threshold:
          emo = String()
          emo.data = 'frustrated'
          self._affect_publisher.publish(emo)
          logger.info('Chatbot perceived emo: {}'.format(emo.data))
          # Currently response is independant of message received so no need to wait
          # Leave it for Opencog to handle responses later on.

      self._response_publisher.publish(String(response))
      logger.info("Ask: {}, answer: {}".format(chat_message.utterance, response))

  # Just repeat the chat message, as a plain string.
  def _echo_callback(self, chat_message):
    message = String()
    message.data = chat_message.utterance
    self._echo_publisher.publish(message)

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('botname', help ='robot name')
  parser.add_argument('-sent', action='store_true', default=False, help='Enable sentiment')
  option, unknown = parser.parse_known_args()

  print 'before chatbot class {}'.format(option.botname)
  logger.info('before chatbot constructor')
  rospy.init_node('chatbot_en')
  chatbot = Chatbot(option.botname)
  print 'after chatbot'
  logger.info("after chatbot")

  if option.sent:
    logger.info("Enable sentiment")
    # by default no sentiment so make active if got arg
    chatbot.sentiment_active(True)
    current=os.path.dirname(os.path.realpath(__file__))
    sent3_file=current+"/../character_aiml/senticnet3.props.csv"
    try:
      chatbot.polarity.load_sentiment_csv(sent3_file)
    except Exception as ex:
      logger.warn("Load sentiment file error {}".format(ex))
      chatbot.sentiment_active(False)
  rospy.spin()
