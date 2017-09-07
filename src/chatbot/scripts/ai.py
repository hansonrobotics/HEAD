#!/usr/bin/env python

import rospy
import os
import logging
import requests
import json
import time
import datetime as dt
import threading
import re
import uuid
import pandas as pd
import random

from jinja2 import Template, Environment, meta

from chatbot.polarity import Polarity
from chatbot.msg import ChatMessage
from std_msgs.msg import String
from audio_stream.msg import audiodata
import dynamic_reconfigure
from dynamic_reconfigure.server import Server
import dynamic_reconfigure.client
from chatbot.cfg import ChatbotConfig
from chatbot.client import Client
from blender_api_msgs.msg import SetGesture
from hr_msgs.msg import TTS

logger = logging.getLogger('hr.chatbot.ai')
HR_CHATBOT_AUTHKEY = os.environ.get('HR_CHATBOT_AUTHKEY', 'AAAAB3NzaC')
HR_CHATBOT_REQUEST_DIR = os.environ.get('HR_CHATBOT_REQUEST_DIR') or \
    os.path.expanduser('~/.hr/chatbot/requests')

def update_parameter(node, param, *args, **kwargs):
    client = dynamic_reconfigure.client.Client(node, *args, **kwargs)
    try:
        client.update_configuration(param)
    except dynamic_reconfigure.DynamicReconfigureParameterException as ex:
        logger.error("Updating {} parameter: {}".format(node, ex))
        return False
    return True

class Console(object):
    def write(self, msg):
        logger.info("Console: {}".format(msg.strip()))

class Chatbot():

    def __init__(self):
        self.botname = rospy.get_param('botname', 'sophia')
        self.client = Client(
            HR_CHATBOT_AUTHKEY, response_listener=self,
            botname=self.botname, stdout=Console())
        self.client.chatbot_url = rospy.get_param(
            'chatbot_url', 'http://localhost:8001')
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
        self.enable = True
        self.mute = False
        self.insert_behavior = False

        self.node_name = rospy.get_name()
        self.output_dir = os.path.join(HR_CHATBOT_REQUEST_DIR,
            dt.datetime.strftime(dt.datetime.now(), '%Y%m%d'))
        if not os.path.isdir(self.output_dir):
            os.makedirs(self.output_dir)
        self.requests_fname = os.path.join(
            self.output_dir, '{}.csv'.format(str(uuid.uuid1())))

        self.input_stack = []
        self.timer = None
        self.delay_response = rospy.get_param('delay_response', False)
        self.recover = False
        self.delay_time = rospy.get_param('delay_time', 5)

        rospy.Subscriber('chatbot_speech', ChatMessage, self._request_callback)
        rospy.Subscriber('speech_events', String, self._speech_event_callback) # robot starts to speak
        rospy.Subscriber('chat_events', String, self._chat_event_callback) # user starts to speak

        rospy.Subscriber('audio_sensors', audiodata, self._audio_sensors_callback)
        self.tts_ctrl_pub = rospy.Publisher(
            'tts_control', String, queue_size=1)

        self._response_publisher = rospy.Publisher(
            'chatbot_responses', TTS, queue_size=1)
        self._suggestion_publisher = rospy.Publisher(
            'chatbot_suggestions', TTS, queue_size=1)

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

        self.btree_publisher = rospy.Publisher(
            '/behavior_switch', String, queue_size=1)

        self._gesture_publisher = rospy.Publisher(
            '/blender_api/set_gesture', SetGesture, queue_size=1)

    def sentiment_active(self, active):
        self._sentiment_active = active

    def ask(self, chatmessages, query=False):
        lang = rospy.get_param('lang', None)
        if lang:
            self.client.lang = lang

        persons = rospy.get_param('/face_recognizer/current_persons', '')
        if persons:
            person = persons.split('|')[0]
            person = person.title()
            self.client.set_context('queryname={}'.format(person))
            logger.info("Set queryname to {}".format(person))
        else:
            self.client.remove_context('queryname')
            logger.info("Remove queryname")

        request_id = str(uuid.uuid1())
        question = ' '.join([msg.utterance for msg in chatmessages])
        logger.info("Asking {}".format(question))
        self.client.ask(question, query, request_id=request_id)
        logger.info("Sent request {}".format(request_id))
        self.write_request(request_id, chatmessages)

    def _speech_event_callback(self, msg):
        if msg.data == 'start':
            self.speech = True
        if msg.data == 'stop':
            rospy.sleep(2)
            self.speech = False

    def _chat_event_callback(self, msg):
        if msg.data.startswith('speechstart'):
            if self.delay_response:
                self.reset_timer()

    def _audio_sensors_callback(self, msg):
        if msg.Speech:
            self.client.cancel_timer()

    def _request_callback(self, chat_message):
        if not self.enable:
            logger.info("Chatbot is disabled")
            return
        if 'shut up' in chat_message.utterance.lower():
            logger.info("Robot's talking wants to be interruptted")
            self.tts_ctrl_pub.publish("shutup")
            rospy.sleep(0.5)
            self._affect_publisher.publish(String('sad'))
            if not self.mute:
                self._response_publisher.publish('Okay', 'en')
            return

        # Handle chatbot command
        cmd, arg, line = self.client.parseline(chat_message.utterance)
        func = None
        try:
            if cmd is not None:
                func = getattr(self.client, 'do_' + cmd)
        except AttributeError as ex:
            pass
        if func:
            try:
                func(arg)
            except Exception as ex:
                logger.error("Executing command {} error {}".format(func, ex))
            return

        # blink that we heard something, request, probability defined in
        # callback
        self._blink_publisher.publish('chat_heard')

        if self.delay_response:
            logger.info("Add input: {}".format(chat_message.utterance))
            self.input_stack.append((time.clock(), chat_message))
            msg = SetGesture()
            msg.name = 'nod-1'
            msg.speed = 1
            msg.magnitude = 1
            self._gesture_publisher.publish(msg)
            self.reset_timer()
        else:
            self.ask([chat_message])

    def reset_timer(self):
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
        self.timer = threading.Timer(self.delay_time, self.process_input)
        self.timer.start()
        logger.info("Timer is reset, {}".format(self.delay_time))

    def process_input(self):
        if not self.input_stack:
            return
        questions = [i[1].utterance for i in self.input_stack]
        question = ' '.join(questions)
        logger.info("Joined input: {}".format(question))
        self.ask([i[1] for i in self.input_stack])
        del self.input_stack[:]

    def write_request(self, request_id, chatmessages):
        rows = []
        columns = ['RequestID', 'Index', 'Source', 'AudioPath', 'Transcript']
        for i, msg in enumerate(chatmessages):
            audio = os.path.basename(msg.extra)
            row = {
                'RequestID': request_id,
                'Index': i,
                'Source': msg.source,
                'AudioPath': audio,
                'Transcript': msg.utterance
            }
            rows.append(row)
        df = pd.DataFrame(rows)
        if not os.path.isfile(self.requests_fname):
            with open(self.requests_fname, 'w') as f:
                f.write(','.join(columns))
                f.write('\n')
        df.to_csv(self.requests_fname, mode='a', index=False, header=False,
            columns=columns)
        logger.info("Write request to {}".format(self.requests_fname))

    def handle_control(self, response):
        t = Template(response)
        if hasattr(t.module, 'delay'):
            delay = t.module.delay
            if not self.delay_response:
                self.recover = True
            param = {
                'delay_response': True,
                'delay_time': delay,
            }
            update_parameter('chatbot', param, timeout=2)
            logger.info("Set delay to {}".format(delay))
        if hasattr(t.module, 'btree'):
            btree = t.module.btree
            if btree in ['btree_on', 'on', 'true', True]:
                self.btree_publisher.publish('btree_on')
                logger.info("Enable btree")
            elif btree in ['btree_off', 'off', 'false', False]:
                self.btree_publisher.publish('btree_off')
                logger.info("Disable btree")
            else:
                logger.warn("Incorrect btree argument, {}".format(btree))

    def on_response(self, sid, response):
        if response is None:
            logger.error("No response")
            return

        if sid != self.client.session:
            logger.error("Session id doesn't match")
            return

        logger.info("Get response {}".format(response))

        for k, v in response.iteritems():
            rospy.set_param('{}/response/{}'.format(self.node_name, k), v)

        text = response.get('text')
        emotion = response.get('emotion')

        orig_text = response.get('orig_text')
        if orig_text:
            try:
                self.handle_control(orig_text)
            except Exception as ex:
                logger.error(ex)
        elif self.recover:
            param = {
                'delay_response': False
            }
            update_parameter('chatbot', param, timeout=2)
            self.recover = False
            logger.info("Recovered delay response")

        # Add space after punctuation for multi-sentence responses
        text = text.replace('?', '? ')
        text = text.replace('_', ' ')
        if self.insert_behavior:
            # no
            pattern=r"(\bnot\s|\bno\s|\bdon't\s|\bwon't\s|\bdidn't\s)"
            text = re.sub(pattern, '\g<1>|shake3| ', text, flags=re.IGNORECASE)

            # yes
            pattern=r'(\byes\b|\byeah\b|\byep\b)'
            text = re.sub(pattern, '\g<1>|nod|', text, flags=re.IGNORECASE)

            # question
            # pattern=r'(\?)'
            # thinks = ['thinkl', 'thinkr', 'thinklu', 'thinkld', 'thinkru', 'thinkrd']
            # random.shuffle(thinks)
            # text = re.sub(pattern, '|{}|\g<1>'.format(thinks[0]), text, flags=re.IGNORECASE)

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
                p = self.polarity.get_polarity(text)
                logger.info('Polarity for "{}" is {}'.format(
                    text.encode('utf-8'), p))
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

        if not self.mute:
            self._blink_publisher.publish('chat_saying')
            if self.mode == 'automatic':
                self._response_publisher.publish(text, 'en')
            elif self.mode == 'semiautomatic':
                self._suggestion_publisher.publish(text, 'en')
            else:
                pass

        if rospy.has_param('{}/context'.format(self.node_name)):
            rospy.delete_param('{}/context'.format(self.node_name))
        context = self.client.get_context()
        context['sid'] = self.client.session
        for k, v in context.iteritems():
            rospy.set_param('{}/context/{}'.format(self.node_name, k), v)
            logger.info("Set param {}={}".format(k, v))

    # Just repeat the chat message, as a plain string.
    def _echo_callback(self, chat_message):
        message = String()
        message.data = chat_message.utterance
        self._echo_publisher.publish(message)

    def reconfig(self, config, level):
        self.sentiment_active(config.sentiment)
        self.client.chatbot_url = config.chatbot_url
        self.enable = config.enable
        if not self.enable:
            self.client.cancel_timer()
        self.delay_response = config.delay_response
        self.delay_time = config.delay_time
        self.client.ignore_indicator = config.ignore_indicator
        if config.set_that:
            self.client.do_said(config.set_that)
            config.set_that = ''

        if config.set_context:
            self.client.set_context(config.set_context)
        self.client.set_marker(config.marker)
        self.mute = config.mute
        self.insert_behavior = config.insert_behavior

        if config.reset_session:
            self.client.reset_session()
            config.reset_session = Fales

        self.mode = config.mode

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
