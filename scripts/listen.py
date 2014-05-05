#!/usr/bin/env python

import os
import rospy
import shlex
import subprocess

from chatbot.msg import ChatMessage

cmd1='sox -r 16000 -t alsa default recording.flac silence 1 0.1 1% 1 1.5 1%'
cmd2='wget -q -U "Mozilla/5.0" --post-file recording.flac --header="Content-Type: audio/x-flac; rate=16000" -O - "http://www.google.com/speech-api/v1/recognize?lang=en-us&client=chromium"'

class ChatbotSpeechRecognizer:
  def __init__(self):
    rospy.init_node('chatbot_recognizer')
    self._speech_publisher = rospy.Publisher('chatbot_speech', ChatMessage)
    self._args2 = shlex.split(cmd2)

  def start(self):
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
      self._recognize_speech()
      rate.sleep()

  def _recognize_speech(self):
    os.system(cmd1)  
    output, error = subprocess.Popen(
      self._args2,
      stdout=subprocess.PIPE,
      stderr= subprocess.PIPE
    ).communicate()

    if not error and len(output)>16:
      a = eval(output)
      if ('hypotheses' in a
          and len(a['hypotheses']) > 0
          and 'confidence' in a['hypotheses'][0]
          and 'utterance' in a['hypotheses'][0]):
        confidence = a['hypotheses'][0]['confidence']
        confidence = confidence * 100
        message = ChatMessage()
        message.utterance = a['hypotheses'][0]['utterance']
        message.confidence = confidence

        self._speech_publisher.publish(message)

def main():
  recognizer = ChatbotSpeechRecognizer()
  recognizer.start() 

if __name__ == '__main__':
  main()
