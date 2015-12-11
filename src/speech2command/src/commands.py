# -*- coding: utf-8 -*-
import logging
import rospy
from chatbot.msg import ChatMessage
from std_msgs.msg import String
from blender_api_msgs.msg import EmotionState, SetGesture
from calc import calculate

logger = logging.getLogger('hr.speech2command.commands')

class BaseCommand(object):
    def parse(self, msg):
        """
        Parse the chat message. Set the command parameters according to
        the message if needed.

        @param msg: instance of ChatMessage
        @return True: the command can recognize and handle the message
                False: otherwise
        """
        return True

    def execute(self):
        """
        Execute the command.

        @return True: the command is successfully executed
                False: otherwise
        """
        return True

class DefaultCommand(BaseCommand):
    def __init__(self):
        self.chatbot_speech_pub = rospy.Publisher(
            'chatbot_speech', ChatMessage, queue_size=1)

    def parse(self, msg):
        self.msg = msg
        return True

    def execute(self):
        self.chatbot_speech_pub.publish(self.msg)
        logger.info("Publish msg to chat conversation {}".format(self.msg))
        return True

class EmotionCommand(BaseCommand):
    # Supported emotions: ['irritated', 'happy', 'recoil', 'surprised', 'sad', 'confused', 'worry', 'bored', 'engaged', 'amused', 'comprehending', 'afraid']
    emotion_command_config = {
        'irritated': u'irritate',
        'happy': u'happy|高兴|开心',
        'recoil': u'recoil',
        'surprised': u'surprise|惊讶',
        'sad': u'sad|悲伤|伤心',
        'confused': u'confuse|疑惑',
        'worry': u'worry|担心',
        'bored': u'bore|无聊',
        'engaged': u'engage',
        'amused': u'amuse',
        'comprehending': u'comprehend',
        'afraid': u'afraid|害怕',
    }

    def __init__(self):
        self.emotion_pub = rospy.Publisher(
            '/blender_api/set_emotion_state', EmotionState, queue_size=1)
        self.chatbot_response_pub = rospy.Publisher(
            'chatbot_responses', String, queue_size=1)
        self.emotion_commands = {}
        for key, value in self.emotion_command_config.iteritems():
            for cmd in value.split('|'):
                if cmd:
                    self.emotion_commands[cmd] = key
        self.emotion = None

    def parse(self, msg):
        text = msg.utterance.decode('utf-8').lower()
        if text in self.emotion_commands:
            self.emotion = self.emotion_commands[text]
            return True
        else:
            return False
        
    def execute(self):
        msg = EmotionState()
        msg.name = self.emotion
        msg.magnitude = 1
        msg.duration = rospy.Duration(6, 0)
        self.emotion_pub.publish(msg)
        lang = rospy.get_param('lang', None)
        if lang == 'en':
            self.chatbot_response_pub.publish(String("Okay."))
        if lang == 'zh':
            self.chatbot_response_pub.publish(String("好的。"))
        logger.info("Set emotion {}".format(self.emotion))
        return True

class GestureCommand(BaseCommand):

    # Supported gestures: ['all', 'amused', 'blink', 'blink-micro', 'blink-relaxed', 'blink-sleepy', 'nod-1', 'nod-2', 'nod-3', 'nod-4', 'nod-5', 'nod-6', 'shake-2', 'shake-3', 'shake-4', 'shake-5', 'shake-6', 'think-brows', 'think-R.UP', 'thoughtful', 'yawn-1']
    gesture_command_config = {
        'blink': u'blink|眨眼',
        'nod-1': u'nod|点头',
        'shake-2': u'shake|摇头',
        'think-brows': u'think|思考',
        'yawn-1': u'yawn|打哈欠'
    }

    def __init__(self):
        self.gesture_pub = rospy.Publisher(
            '/blender_api/set_gesture', SetGesture, queue_size=1)
        self.chatbot_response_pub = rospy.Publisher(
            'chatbot_responses', String, queue_size=1)
        self.gesture_commands = {}
        for key, value in self.gesture_command_config.iteritems():
            for cmd in value.split('|'):
                if cmd:
                    self.gesture_commands[cmd] = key
        self.gesture = None

    def parse(self, msg):
        text = msg.utterance.decode('utf-8').lower()
        if text in self.gesture_commands:
            self.gesture = self.gesture_commands[text]
            return True
        else:
            return False
        
    def execute(self):
        msg = SetGesture()
        msg.name = self.gesture
        msg.repeat = 1
        msg.speed = 1
        msg.magnitude = 1
        self.gesture_pub.publish(msg)
        lang = rospy.get_param('lang', None)
        if lang == 'en':
            self.chatbot_response_pub.publish(String("Okay."))
        if lang == 'zh':
            self.chatbot_response_pub.publish(String("好的。"))
        logger.info("Set gesture {}".format(self.gesture))
        return True

class Calculator(BaseCommand):

    def __init__(self):
        self.chatbot_response_pub = rospy.Publisher(
            'chatbot_responses', String, queue_size=1)
        self.ans = None

    def parse(self, msg):
        """
        example: "one plus two minus three"
        """
        text = msg.utterance.decode('utf-8').lower()
        try:
            self.ans = calculate(text)
            return True
        except:
            return False

    def execute(self):
        self.chatbot_response_pub.publish(String(str(self.ans)))
        logger.info("Result is {}".format(self.ans))
        return True

command_chain = [EmotionCommand(), GestureCommand(), Calculator(), DefaultCommand()]
