#!/usr/bin/env python

import rospy
import os
import logging
import requests
import json
import time
import threading
import re
import random
from functools import wraps
from operator import itemgetter

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

        #self.input_stack = []
        #self.condition = threading.Condition()
        #self.respond_worker = threading.Thread(target=self.process_input)
        #self.respond_worker.daemon = True
        #self.respond_worker.start()
        #self.delay_response = rospy.get_param('delay_response', False)
        #self.delay_time = rospy.get_param('delay_time', 2)

        # initialize word buffer and timeout timer
        self.words = []
        self.timer = None

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

    def retry(times):
        def wrap(f):
            @wraps(f)
            def wrap_f(*args):
                for i in range(times):
                    try:
                        return f(*args)
                    except Exception as ex:
                        logger.error(ex)
                        self = args[0]
                        self.session = self.start_session()
                        continue
            return wrap_f
        return wrap

    @retry(3)
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

        #if self.delay_response:
        #    with self.condition:
        #        logger.info("Add input: {}".format(chat_message.utterance))
        #        self.input_stack.append((time.clock(), chat_message))
        #        self.condition.notify_all()
        #else:

        # only pass as single question
        self.respond(chat_message.utterance)

    #def process_input(self):
    #    while True:
    #        time.sleep(0.1)
    #        with self.condition:
    #            if not self.input_stack:
    #                continue
    #            num_input = len(self.input_stack)
    #            questions = [i[1].utterance for i in self.input_stack]
    #            question = ' '.join(questions)
    #            logger.info("Current input: {}".format(question))
    #            if len(question) < 10:
    #                self.condition.wait(self.delay_time)
    #                if len(self.input_stack) > num_input:
    #                    continue
    #            self.respond(questions)
    #            del self.input_stack[:]

    def score_question(self,question,lang):

        # this calculates a score for the given question, by querying the pattern trace from the chatbot server

        # get response and category, but don't execute the question
        response = self.get_response(question,lang,True)

        if not "trace" in response:
            logger.info("no trace found")
            return 0

        # lift patterns from the trace (assuming it's always trace[3], where trace is response["trace"] split by commas)
        patterns = []
        for trace in response["trace"]:
            patterns.append(trace.split(',')[3])
        
        logger.info("question:" + question + "   patterns:" + str(patterns))

        # patterns with * and _ are catch-all patterns that somehow try to generalize the incoming user input
        # the idea is that patterns with * or _ are less preferable than patterns with complete matches

        # count number of words in response and count number of * in pattern, and rank according to these counts
        
        # take all patterns in the trace as one
        total_words = 0.0
        total_wildcards = 0.0
        total_all = 0.0
        total_full_patterns = 0.0
        for pattern in patterns:
            words = pattern.split()
            no_wildcards = True
            for word in words:
                total_all = total_all + 1.0
                if (word == "*") or (word == "_"):
                    total_wildcards = total_wildcards + 1.0
                    no_wildcards = False
                else:
                    total_words = total_words + 1.0
            # special case: if one of the patterns has no wildcards, it is preferred
            if no_wildcards:
                total_full_patterns = total_full_patterns + 1.0

        # diagnose possible illnesses of the patterns
        # patterns that are very short tend to fit a general response
        very_short = False
        if total_words < 3:
            very_short = True

        # pattern traces that have one or more wildcards are a bit problematic
        small_match = False
        if total_wildcards >= 1:
            small_match = True

        # pattern traces that have two or more wildcards are more problematic 
        very_hard = False
        if total_wildcards >= 2:
            very_hard = True

        # if one of the patterns in the trace had no wildcards, this is a good thing
        did_resolve = False
        if total_full_patterns >= 1:
            did_resolve = True

        # determine fitness of the patterns according to the illnesses
        score = 0
        if did_resolve: # if one of the patterns in the trace resolved the question, this could pass
            score = 3
        elif very_short and very_hard: # if the patterns are very short and have a lot of wildcards, they are very bad
            score = 0
        elif very_short and small_match: # if the patterns are very short and have atleast one wildcard, they are a little less bad
            score = 1
        elif very_hard or small_match or very_short: # if the patterns are very short or have atleast one wildcard, they are almost ok
            score = 2
        else:
            score = 4

        #print "    final score:",score
            
        return score


    def get_question_scores(self,question,lang):

        logger.info("incoming text: " + question)

        # pre-test the text for number of words
        sentence = question.split()
        num_of_words = len(sentence)

        # it seems preselection works best for very short questions (not
        # patterns), like "you?", "do you", "is that true?", "I think so",
        # "yes", etc.
        # for that purpose, we cut away all questions that are longer
        # than 2 words, and score high enough

        if (num_of_words > 2) and (self.score_question(question,lang) > 3):
            result = {"value":4,"question":question}
            return [result]

        # what is left are difficult to answer (ill patterns) or short question chunks

        # one of the reasons might be that the question was interrupted by
        # the STT system, and we need to combine with previously said chunks
        # as well

        for word in sentence:
            self.words.append(word)

        logger.info("trying multiple combinations")

        # reset statistics
        scores = []

        # try all possible options from back to front and fill up scores[]
        for i in range(0,len(self.words)):

            # build question
            attempt = ""
            for k in range(i,len(self.words)):
                attempt = attempt + " " + self.words[k]

            # find the score for this question
            score = { "question": attempt, "value": 0 }
            score["value"] = self.score_question(attempt,lang)

            # and add to the list
            scores.append(score)

        # sort the list
        results = sorted(scores,key=itemgetter("value"),reverse=True)

        # debug output
        for result in results:
            logger.info(str(result["value"]) + ": " + result["question"])

        return results


    def finally_respond(self,question,lang):

        # this finally processes the question and outputs the answer, like before

        answer = self.get_response(question,lang,False)
        response = answer["text"]
        emotion = answer["emotion"]
        botid = answer["botid"]

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

        # since this was processed, we don't need any previously stored words anymore,
        # and we can switch off the reset timer
        self.words = []
        if self.timer is not None:
            self.timer.shutdown()
        self.timer = None


    def respond(self,question):

        # first get a sorted list of scores for the words in this question,
        # where needed combined with previously stored words
        lang = rospy.get_param('lang', None)
        results = self.get_question_scores(question,lang)

        # if the best one is reasonable, publish it
        if results[0]["value"] > 3:
            self.finally_respond(results[0]["question"],lang)
        else:

            logger.info("nothing reasonable found, 'go on'...")
    
            # output something enticing to get the user to talk more

            # the shorter the response, the more ambivalent, so the more
            # it 'fits' to the user's idea of what is going on, therefore,
            # find the shortest question and use that in the enticing comment
            least_i = -1
            least = 10
            i = 0
            num_of_threes = 0
            for result in results:
                if result["value"] >= 2:
                    count = len(result["question"].split())
                    if (least_i == -1) or (count < least):
                        least_i = i
                        least = count
                    if result["value"] == 3:
                        num_of_threes = num_of_threes + 1
                i = i + 1

            # added later: if one or more of the patterns were slightly dodgy,
            # but acceptable, respond with them
    	    if num_of_threes > 0:
        		self.finally_respond(results[0]["question"],lang)
        		return

            # take the Smallest Almost Reasonable Question
            sarq = results[least_i]["question"]

            # randomly choose how to proceed
            choice = random.randint(0,6)

            if choice == 0:  # get user to reformulate in more clear sentence
                self._response_publisher.publish(String("I'm not sure I understand " + sarq + "."))
            elif choice == 1:  # get user to explain more about the topic
                self._response_publisher.publish(String("Tell me more about " + sarq + "."))
            elif choice == 2:  # get user to continue talking about the topic
                self._response_publisher.publish(String("Go on, what " + sarq + "."))
            elif choice == 3:  # get user to reformulate the topic
                self._response_publisher.publish(String("What do you mean by " + sarq + "?"))
            elif choice == 4:  # take the AIML response from the chatbot (could be funny anyway)
                self.finally_respond(results[0]["question"],lang)
                return
            else:  # indicate general feeling of cluelessness about the topic
                self._response_publisher.publish(String("What " + sarq + "?"))

            # reset timer, the next words from the user could help, but don't wait too long
            if self.timer is not None:
                self.timer.shutdown()
            self.timer = rospy.Timer(rospy.Duration(5),self.reset_tick,True)

    def reset_tick(self,event):
        # the user waited too long, so we can't use the currently stored words anymore
        logger.info("reset, silence was too long")
        self.words = []
        self.timer.shutdown()
        self.timer = None

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
        self.delay_time = config.delay_time
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
