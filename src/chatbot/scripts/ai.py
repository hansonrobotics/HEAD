#!/usr/bin/env python

import aiml
import rospy
import os
import sys
import time
# csv and itertools for #sentiment
import csv
from itertools import izip
from chatbot.msg import ChatMessage
from std_msgs.msg import String
""
#from blender_api_msgs.msg import EmotionState
import logging
import random
import argparse
#from rigControl.actuators import sleep as nb_sleep

logger = logging.getLogger('hr.chatbot.ai')

class Chatbot():
  def __init__(self, botname):
    self._generic = aiml.Kernel()
    self._character = aiml.Kernel()
    print 'calling init botname', botname
    self.initialize(botname)
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
    self._polarity={}
    self._polarity_threshold=0.2
    # a small dictionary of terms which negate polarity
    self._negates={'not':1,'don\'t':1,'can\'t':1,'won\'t':1,'isn\'t':1,'never':1}
    #
    rospy.Subscriber('chatbot_speech', ChatMessage, self._request_callback)
    # add callback to let sentiment analysis possibly set emotion

    rospy.Subscriber('chatbot_speech', ChatMessage, self._affect_perceive_callback)

    self._response_publisher = rospy.Publisher('chatbot_responses', String, queue_size=1)

    # send communication non-verbal blink message to behavior
    self._blink_publisher = rospy.Publisher('chatbot_blink',String,queue_size=1)

    # Perceived emotional content; and emotion to express
    # Perceived: based on what chatbot heard, this is how robot should
    # feel.  Expressed: the emotional content that the chatbot should
    # put into what it says.
    self._affect_publisher = rospy.Publisher(
      'chatbot_affect_perceive',
      String, queue_size=1
    )
    # rospy.Subscriber('chatbot_affect_express', EmotionState,
    #     self._affect_express_callback)

    # Echo chat messages as plain strings.
    self._echo_publisher = rospy.Publisher('perceived_text', String, queue_size=1)
    rospy.Subscriber('chatbot_speech', ChatMessage, self._echo_callback)

._character.learn(character_dir)

    propname=current+'/../character_aiml/' + botname + '.properties'
    try:  def initialize(self, botname):
    rospy.init_node('chatbot_en')
      # read properties
    current=os.path.dirname(os.path.realpath(__file__))
    print current
    currstr=current+"/aiml/standard/*.aiml"
    self._generic.learn(currstr)

    # this is from current hanson chat set but not character specific
    genstrx=current+"/../generic_aiml/*.xml"
    genstra=current+"/../generic_aiml/*.aiml"
    print 'loading generic from',genstrx
    self._generic.learn(genstrx)
    self._generic.learn(genstra)
    #
    character_dir="../character_aiml/"+ botname+"*.xml"
    print 'loading character from', character_dir

    self
        f=open(propname)
        lines= f.readlines()
        for line in lines:
          parts = line.split('=')

          key = parts[0].strip()
          value = parts[1].strip()
          self._character.setBotPredicate(key, value)
          self._generic.setBotPredicate(key, value)
          print 'loading props', key, value
          f.close()
    except:
      logger.warn("couldn't open property file", propname)
      #self._kernel.learn(os.sep.join([aimldir, '*.aiml']))

    logger.info('Done initializing chatbot.')

  def sentiment_active(self):
    self._sentiment_active=True

  def load_sentiment_csv(self,sent3_file):
    logger.warn('loading sentiment')
    # for some reason doesn't work to open file here, only in main?
    reader=csv.reader(sent3_file)
    sent3=list(reader)
    # delete header keep phrases over threshold
    del sent3[0]
    # keep phrases over threshold
    for phrase in sent3:
      # 1 is phrase string with spaces, 6 is polarity
      if abs(float(phrase[6])) > 0.1:
        self._polarity[phrase[1]]=float(phrase[6])
    logger.warn("Loaded {} items".format(len(self._polarity)))

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

      character_match=self._character.respond(chat_message.utterance)
      #logger.warn('UTTERANCE', chat_message.utterance)
      if len(character_match)>0:
        response =character_match
      else:
        respons= self._generic.respond(chat_message.utterance)
      # Add space after punctuation for multi-sentence responses
      response = response.replace('?','? ')
      response = response.replace('.','. ')

      # if sentiment active save state and wait for affect_express to publish response
      # otherwise publish and let tts handle it
    self._response_buffer=response
    if self._sentiment_active:
      self._state = 'wait_emo'
    else:
      logger.warn('Sentiment off, immediate publish to tts')
      message = String()
      message.data = response
      self._response_publisher.publish(message)
      self._state = 'wait_client'
    logger.info("Ask: {}, answer: {}".format(chat_message.utterance, response))

  # Just repeat the chat message, as a plain string.
  def _echo_callback(self, chat_message):
    message = String()
    message.data = chat_message.utterance
    self._echo_publisher.publish(message)

  # Tell the world the emotion that the chatbot is perceiving.
  # Use the blender_api_msgs/EmotionState messae type to
  # describe the perceived emotion.  Argument is just a string.
  def _affect_perceive_callback(self, chat_message):

    if chat_message.confidence >= 50 and self._sentiment_active:
      polarity=self._get_polarity(chat_message.utterance)
      polmsg='got polarity '+ str(polarity) + ' for ' + chat_message.utterance
      logger.info(polmsg)
      emo = String()
      # change emotion if polarity magnitude exceeds threshold defined in constructor
      # otherwise let top level behaviors control
      if polarity>self._polarity_threshold:
        emo.data = 'happy'
        self._affect_publisher.publish(emo)
        logger.info('Chatbot perceived emo:'+emo.data)
        # Currently response is independant of message received so no need to wait
        # Leave it for Opencog to handle responses later on.
        self._affect_express_callback()

      elif polarity< 0 and abs(polarity)> self._polarity_threshold:
        emo = String()
        emo.data = 'frustrated'
        self._affect_publisher.publish(emo)
        logger.info('Chatbot perceived emo:'+emo.data)
        # Currently response is independant of message received so no need to wait
        # Leave it for Opencog to handle responses later on.
        self._affect_express_callback()
      else:
        # if no or below threshold affect, publish message so some response is given.

        if self._state == 'wait_emo':
          message = String()
          message.data = self._response_buffer
          # response to tts
          self._response_publisher.publish(message)
          self._state='wait_client'

  # This is the emotion that the chatbot should convey.
  # affect_message is of type blender_api_msgs/EmotionState
  # Fields are name (String) magnitude (float), duration (time)
  def _affect_express_callback(self, affect_message=None):
    #

    #logger.info("Chatbot wants to express: " + affect_message.name+" state='"+self._state)
    # currently general_behavior publishes set emotion and tts listens
    # here we could call some smart markup process instead of letting tts
    # do default behavior.
    # any emotion change will now force tts
    # TODO pass .cfg speech hesitation interval in affect message
    # Will leave for OpenCog behavior tree to decide for now.
    hesitation=random.uniform(0.1,0.4)
    time.sleep(hesitation)
    if self._state == 'wait_emo':
      message = String()
      message.data = self._response_buffer
      self._response_publisher.publish(message)
      self._state='wait_client'

  # Get the polarity (valence) of a response using sentic.net database
  # This should only be called if sentiment_active is true
  def _get_polarity(self, response):
    # break into words
    not_found=0
    average=0.0
    extreme=0.0
    polarity_list=[]
    negate=1

    # strip punctuation prior to word search
    response=response.rstrip('.?!')
    words=response.split()
    for word in words:
      # check if word in negates
      if self._negates.has_key(word):
        negate=-1
      # check if words in sentic
      if word in self._polarity:
        polarity_list.append(self._polarity[word])
        logger.warn(word+' '+str(self._polarity[word]))
      else:
        not_found+=1
  
    # check adjacent pairs as well

    pairs=[' '.join(pair) for pair in izip(words[:-1], words[1:])]
    for pair in pairs:
      if pair in self._polarity:
        polarity_list.append(self._polarity[pair])
        logger.warn(pair+' '+str(self._polarity[pair]))
      else:
        not_found+=1

    # return average and extrema of words in response
    # This is a very simple function that should be improved
    # probably weighting first half of responses better
    if len(polarity_list)>0:
      average=sum(polarity_list)/float(len(polarity_list))
      if abs(max(polarity_list))>abs(min(polarity_list)):
         extreme=max(polarity_list)
      else:
        extreme=min(polarity_list)
      return negate*(average+extreme)/2.0
    else:
      return 0.0

def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('botname', help ='robot name')
  parser.add_argument('-sent', action='store_true', default=False, help='Enable sentiment')

  option, unknown = parser.parse_known_args()
  print 'before chatbot class {}'.format(option.botname)
  logger.info('before chatbot constructor')
  chatbot = Chatbot(option.botname)
  print 'after chatbot'
  logger.info("after chatbot")
  if unknown:
    logger.warn("Unknown options {}".format(unknown))

  if option.sent:
    logger.info("Enable sentiment")
    # by default no sentiment so make active if got arg
    chatbot.sentiment_active()
    current=os.path.dirname(os.path.realpath(__file__))
    #sent3_file=os.path.join(option.botname, 'senticnet3.props.csv')
    sent3_file=current+"../character_aiml"
    try:
      sent_f=open(sent3_file,'r')
      chatbot.load_sentiment_csv(sent_f)
    except Exception as ex:
      logger.warn("Load sentiment file error {}".format(ex))
      chatbot._sentiment_active=False
  rospy.spin()
if __name__ == '__main__':
  main()
