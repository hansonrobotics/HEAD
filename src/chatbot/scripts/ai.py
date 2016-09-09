#!/usr/bin/env python

import rospy
import os
import logging
import requests
import json
import time
import threading
import re

from chatbot.polarity import Polarity
from chatbot.msg import ChatMessage
from std_msgs.msg import String
from dynamic_reconfigure.server import Server
from chatbot.cfg import ChatbotConfig
from chatbot.client import get_default_username

logger = logging.getLogger('hr.chatbot.ai')
VERSION = 'v1.1'
key = 'AAAAB3NzaC'
trace_pattern = re.compile(
    r'../(?P<fname>.*), (?P<tloc>\(.*\)), (?P<pname>.*), (?P<ploc>\(.*\))')


class Chatbot():

    def __init__(self):
        self.chatbot_url = rospy.get_param(
            'chatbot_url', 'http://localhost:8001')
        self.botname = rospy.get_param('botname', 'sophia')
        self.user = get_default_username()
        while not self.ping():
            logger.info("Ping server")
            time.sleep(1)
        self.session = self.start_session()

        # chatbot now saves a bit of simple state to handle sentiment analysis
        # after formulating a response it saves it in a buffer if S.A. active
        # It has a simple state transition - initialized in wait_client
        # after getting client if S.A. active go to wait_emo
        # in affect_express call back publish response and reset to wait_client
        self._response_buffer = ''
        self._state = 'wait_client'
        # argumment must be  to activate sentiment analysis
        self._sentiment_active = False
        # sentiment dictionary
        self.polarity = Polarity()
        self._polarity_threshold = 0.2
        self.speech = False

        self.input_stack = []
        self.condition = threading.Condition()
        self.respond_worker = threading.Thread(target=self.process_input)
        self.respond_worker.daemon = True
        self.respond_worker.start()
        self.delay_response = rospy.get_param('delay_response', False)

        rospy.Subscriber('chatbot_speech', ChatMessage, self._request_callback)
        rospy.Subscriber('speech_events', String, self._speech_event_callback)
        self.tts_ctrl_pub = rospy.Publisher(
            'tts_control', String, queue_size=1)

        self._response_publisher = rospy.Publisher(
            'chatbot_responses', String, queue_size=1)

        # send communication non-verbal blink message to behavior
        self._blink_publisher = rospy.Publisher(
            'chatbot_blink', String, queue_size=1)

        # Perceived emotional content; and emotion to express
        # Perceived: based on what chatbot heard, this is how robot should
        # feel.  Expressed: the emotional content that the chatbot should
        # put into what it says.
        self._affect_publisher = rospy.Publisher(
            'chatbot_affect_perceive', String, queue_size=1)

        # Echo chat messages as plain strings.
        self._echo_publisher = rospy.Publisher(
            'perceived_text', String, queue_size=1)
        rospy.Subscriber('chatbot_speech', ChatMessage, self._echo_callback)
        rospy.set_param('node_status/chatbot', 'running')

    def ping(self):
        try:
            r = requests.get('{}/{}/ping'.format(self.chatbot_url, VERSION))
            response = r.json().get('response')
            if response == 'pong':
                return True
        except Exception:
            return False

    def start_session(self):
        params = {
            "Auth": key,
            "botname": self.botname,
            "user": self.user
        }
        r = requests.get('{}/{}/start_session'.format(
            self.chatbot_url, VERSION), params=params)
        ret = r.json().get('ret')
        if r.status_code != 200:
            raise Exception("Request error: {}\n".format(r.status_code))
        sid = r.json().get('sid')
        logger.info("Start new session {}".format(sid))
        return sid

    def sentiment_active(self, active):
        self._sentiment_active = active

    def get_response(self, question, lang, query=False):
        params = {
            "question": "{}".format(question),
            "session": self.session,
            "lang": lang,
            "Auth": key,
            "query": query,
        }
        r = requests.get('{}/{}/chat'.format(self.chatbot_url, VERSION),
                         params=params)
        ret = r.json().get('ret')
        if r.status_code != 200:
            logger.error("Request error: {}".format(r.status_code))

        if ret != 0:
            logger.error("QA error: error code {}, botname {}, question {}".format(
                ret, self.botname, question))
            raise Exception("QA Error: {}".format(ret))

        response = r.json().get('response', {})

        return response

    def _speech_event_callback(self, msg):
        if msg.data == 'start':
            self.speech = True
        if msg.data == 'stop':
            rospy.sleep(2)
            self.speech = False

    def _request_callback(self, chat_message):
        if not self.enable:
            logger.info("Chatbot is disabled")
            return
        if 'shut up' in chat_message.utterance.lower():
            logger.info("Robot's talking wants to be interruptted")
            self.tts_ctrl_pub.publish("shutup")
            rospy.sleep(0.5)
            self._response_publisher.publish(String('Okay'))
            self._affect_publisher.publish(String('sad'))
            return

        # blink that we heard something, request, probability defined in
        # callback
        self._blink_publisher.publish('chat_heard')

        if chat_message.confidence < 50:
            self._response_publisher.publish('Could you say that again?')
            return

        if self.delay_response:
            with self.condition:
                logger.info("Add input: {}".format(chat_message.utterance))
                self.input_stack.append((time.clock(), chat_message))
                self.condition.notify_all()
        else:
            self.respond([chat_message.utterance])

    def process_input(self):
        while True:
            with self.condition:
                if not self.input_stack:
                    continue
                num_input = len(self.input_stack)
                questions = [i[1].utterance for i in self.input_stack]
                question = ' '.join(questions)
                logger.info("Current input: {}".format(question))
                if len(question) < 10:
                    self.condition.wait(3)
                    if len(self.input_stack) > num_input:
                        continue
                self.respond(questions)
                del self.input_stack[:]
            time.sleep(0.1)

    def respond(self, questions):
        lang = rospy.get_param('lang', None)
        for question in questions:
            tmp_answer = self.get_response(question, lang, True)
            traces = tmp_answer.get('trace')
            if traces:
                pattern = [trace_pattern.match(trace).group('pname')
                           for trace in traces]
                logger.info("Question {}, Pattern {}".format(
                    question, ' '.join(pattern)))

        question = ' '.join(questions)
        try:
            answer = self.get_response(question, lang)
        except Exception:
            self.session = self.start_session()
            answer = self.get_response(question, lang)

        response = answer.get('text')
        emotion = answer.get('emotion')
        botid = answer.get('botid')

        # Add space after punctuation for multi-sentence responses
        response = response.replace('?', '? ')
        response = response.replace('.', '. ')
        response = response.replace('_', ' ')

        # if sentiment active save state and wait for affect_express to publish response
        # otherwise publish and let tts handle it
        if self._sentiment_active:
            emo = String()
            if emotion:
                emo.data = emotion
                self._affect_publisher.publish(emo)
                rospy.loginfo(
                    '[#][PERCEIVE ACTION][EMOTION] {}'.format(emo.data))
                logger.info('Chatbot perceived emo: {}'.format(emo.data))
            else:
                p = self.polarity.get_polarity(response)
                logger.info('Polarity for "{}" is {}'.format(
                    response.encode('utf-8'), p))
                # change emotion if polarity magnitude exceeds threshold defined in constructor
                # otherwise let top level behaviors control
                if p > self._polarity_threshold:
                    emo.data = 'happy'
                    self._affect_publisher.publish(emo)
                    rospy.loginfo(
                        '[#][PERCEIVE ACTION][EMOTION] {}'.format(emo.data))
                    logger.info(
                        'Chatbot perceived emo: {}'.format(emo.data))
                    # Currently response is independant of message received so no need to wait
                    # Leave it for Opencog to handle responses later on.
                elif p < 0 and abs(p) > self._polarity_threshold:
                    emo.data = 'frustrated'
                    self._affect_publisher.publish(emo)
                    rospy.loginfo(
                        '[#][PERCEIVE ACTION][EMOTION] {}'.format(emo.data))
                    logger.info(
                        'Chatbot perceived emo: {}'.format(emo.data))
                    # Currently response is independant of message received so no need to wait
                    # Leave it for Opencog to handle responses later on.

        self._blink_publisher.publish('chat_saying')
        self._response_publisher.publish(String(response))
        logger.info("Ask: {}, answer: {}, answered by: {}".format(
            question, response.encode('utf-8'), botid))

    # Just repeat the chat message, as a plain string.
    def _echo_callback(self, chat_message):
        message = String()
        message.data = chat_message.utterance
        self._echo_publisher.publish(message)

    def reconfig(self, config, level):
        self.sentiment_active(config.sentiment)
        if self.chatbot_url != config.chatbot_url:
            self.chatbot_url = config.chatbot_url
            self.session = self.start_session()
        self.enable = config.enable
        self.delay_response = config.delay_response
        return config

if __name__ == '__main__':
    rospy.init_node('chatbot')
    bot = Chatbot()
    from rospkg import RosPack
    rp = RosPack()
    data_dir = os.path.join(rp.get_path('chatbot'), 'scripts/aiml')
    sent3_file = os.path.join(data_dir, "senticnet3.props.csv")
    bot.polarity.load_sentiment_csv(sent3_file)
    Server(ChatbotConfig, bot.reconfig)
    rospy.spin()
