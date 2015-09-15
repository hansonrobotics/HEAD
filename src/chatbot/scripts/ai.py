#!/usr/bin/env python

import aiml
import rospy
import os
import sys
# csv and itertools for sentiment
import csv
from itertools import izip

from chatbot.msg import ChatMessage
from std_msgs.msg import String
from blender_api_msgs.msg import EmotionState


class Chatbot():
  def __init__(self):
    self._kernel = aiml.Kernel()
    # chatbot now saves a bit of simple state to handle sentiment analysis
    # after formulating a response it saves it in a buffer if S.A. active
    # It has a simple state transition - initialized in wait_client
    # after getting client if S.A. active go to wait_emo
    #  in affect_express call back publish response and reset to wait_client
    self._response_buffer = ''
    self._state = 'wait_client'
    # argumment must be given to activate sentiment analysis
    self._sentiment_active=False
    # sentiment dictionary
    self._polarity={}
    self._polarity_threshold=0.2
    # a small dictionary of terms which negate polarity
    self._negates={'not':1,'don\'t':1,'can\'t':1,'won\'t':1,'isn\'t':1,'never':1}
    #

    rospy.init_node('chatbot_ai')
    rospy.Subscriber('chatbot_speech', ChatMessage, self._request_callback)
    # add callback to let sentiment analysis possibly set emotion

    rospy.Subscriber('chatbot_speech', ChatMessage, self._affect_perceive_callback)

    self._response_publisher = rospy.Publisher(
      'chatbot_responses', String, queue_size=1
    )

    # Perceived emotional content; and emotion to express
    # Perceived: based on what chatbot heard, this is how robot should
    # feel.  Expressed: the emotional content that the chatbot should
    # put into what it says.
    self._affect_publisher = rospy.Publisher(
      'chatbot_affect_perceive',
      String, queue_size=1
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

  def sentiment_active(self):
    self._sentiment_active=True

  def load_sentiment_csv(self,sent3_file):
    rospy.logwarn('loading sentiment')
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

    rospy.logwarn("loaded "+len(self._polarity)+"items")

  def _request_callback(self, chat_message):
    response = ''
    if chat_message.confidence < 50:
      response = 'Could you say that again?'
    else:
      response = self._kernel.respond(chat_message.utterance)
      # Add space after punctuation for multi-sentence responses
      response = response.replace('?','? ')

    # if sentiment active save state and wait for affect_express to publish response
    # otherwise publish and let tts handle it
    self._response_buffer=response
    if self._sentiment_active:
      self._state = 'wait_emo'
    else:
      rospy.logwarn('sentiment off, immediate publish to tts')
      message = String()
      message.data = response
      self._response_publisher.publish(message)
      self._state = 'wait_client'

  # Tell the world the emotion that the chatbot is perceiving.
  # Use the blender_api_msgs/EmotionState messae type to
  # describe the perceived emotion.  Argument is just a string.
  def _affect_perceive_callback(self, chat_message):

    if chat_message.confidence >= 50 and self._sentiment_active:
      polarity=self._get_polarity(chat_message.utterance)
      polmsg='got polarity '+ str(polarity) + ' for ' + chat_message.utterance
      rospy.loginfo(polmsg)
      emo = String()
      # change emotion if polarity magnitude exceeds threshold defined in constructor
      # otherwise let top level behaviors control
      if polarity>self._polarity_threshold:
        emo.data = 'happy'
        self._affect_publisher.publish(emo)
        rospy.loginfo('Chatbot perceived emo:'+emo.data)
      elif polarity< 0 and abs(polarity)> self._polarity_threshold:
        emo = String()
        emo.data = 'frustrated'
        self._affect_publisher.publish(emo)
        rospy.loginfo('Chatbot perceived emo:'+emo.data)
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
  def _affect_express_callback(self, affect_message):
    #

    rospy.loginfo("Chatbot wants to express: " + affect_message.name+" state='"+self._state)
    # currently general_behavior publishes set emotion and tts listens
    # here we could call some smart markup process instead of letting tts
    # do default behavior.
    # any emotion change will now force tts
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
        rospy.logwarn(word+' '+str(self._polarity[word]))
      else:
        not_found+=1
  
    # check adjacent pairs as well

    pairs=[' '.join(pair) for pair in izip(words[:-1], words[1:])]
    for pair in pairs:
      if pair in self._polarity:
        polarity_list.append(self._polarity[pair])
        rospy.logwarn(pair+' '+str(self._polarity[pair]))
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
    return (average+extreme)/2.0

    return negate*(average+extreme)/2.0

def main():
  chatbot = Chatbot()
  #sys.argv=['ai.py','/catkin_ws/src/chatbot/aiml/','-sent']
  aiml_dir = sys.argv[1]

  if sys.argv[2]=='-sent':
    rospy.logwarn("got -sent")
    # by default no sentiment so make active if got arg
    chatbot.sentiment_active()
    sent3_file=aiml_dir+'senticnet3.props.csv'
    try:
      sent_f=open(sent3_file,'r')
      chatbot.load_sentiment_csv(sent_f)
    except:
      rospy.logwarn("exception here even though file read OK...")
    #rospy.logwarn('Done loading '+len(self._polarity)+' sentiment polarity items.')
  # this needs to be at end to see ros msgs in load_sentiment ( spin is hidden inside? )
  chatbot.initialize(aiml_dir)
if __name__ == '__main__':
  main()
