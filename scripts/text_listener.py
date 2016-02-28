#!/usr/bin/env python

import rospy
import os
import sys
import time
import threading

from chatbot.msg import ChatMessage
from std_msgs.msg import String
import logging

logger = logging.getLogger('hr.chatbot.text_listener')

class TextListener(object):

    def __init__(self):
        rospy.Subscriber('chatbot_text', String, self._cb)
        self.pub = rospy.Publisher('speech', ChatMessage, queue_size=1)
        self.text_cache = ''
        self.update_time = None
        self.timeout = rospy.get_param('timeout', 2)

    def _cb(self, msg):
        self.update_time = time.time()
        self.text_cache += msg.data
        threading.Timer(self.timeout, self.send).start()
        logger.info('Length {}, Append {}'.format(len(self.text_cache), msg.data))

    def send(self):
        dragon_enabled = rospy.get_param('webui/speech_recognition') == 'dragon'
        text = self.text_cache.strip()

        if text and dragon_enabled and time.time() - self.update_time >= self.timeout:
            message = ChatMessage()
            message.utterance = text
            message.confidence = 100
            self.pub.publish(message)
            self.text_cache = ''

if __name__ == '__main__':
    rospy.init_node('text_listener')
    TextListener()
    rospy.spin()
