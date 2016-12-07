# -*- coding: utf-8 -*-
import logging
import rospy
import yaml
import re
import string
from chatbot.msg import ChatMessage
from std_msgs.msg import String
from blender_api_msgs.msg import EmotionState, SetGesture, Target, SomaState
from calc import calculate

logger = logging.getLogger('hr.speech2command.commands')

class BaseCommand(object):

    def __init__(self):
        self.active = True
        self.type = 'default'

    def parse(self, msg):
        """
        Parse the chat message. Set the command parameters according to
        the message if needed.

        @param msg: instance of ChatMessage
        @return True: the command can recognize and handle the message
                False: otherwise
        """
        return False

    def execute(self):
        """
        Execute the command.

        @return True: the command is successfully executed
                False: otherwise
        """
        return True

class ForwardCommand(BaseCommand):
    def __init__(self, forward_topic_name):
        super(ForwardCommand, self).__init__()
        self.forward_topic_name = forward_topic_name
        self.chatbot_speech_pub = rospy.Publisher(
            self.forward_topic_name, ChatMessage, queue_size=1)

    def parse(self, msg):
        self.msg = msg
        return True

    def execute(self):
        self.chatbot_speech_pub.publish(self.msg)
        logger.info("Forward msg {} to {}".format(
            self.msg, self.forward_topic_name))
        return True

class MotionCommand(BaseCommand):
    MSGTYPES = {
        'gesture': SetGesture,
        'emotion': EmotionState,
        'turn': Target,
        'look': Target,
    }
    def __init__(self, yaml_config):
        super(MotionCommand, self).__init__()
        self.chatbot_response_pub = rospy.Publisher(
            'chatbot_responses', String, queue_size=1)
        self.motion_defs = []
        with open(yaml_config) as f:
            self.config = yaml.load(f)
        self._build_commands()
        self.matched_motion = None
        self.gesture_pub = rospy.Publisher(
            '/blender_api/set_gesture', SetGesture, queue_size=1)
        self.emotion_pub = rospy.Publisher(
            '/blender_api/set_emotion_state', EmotionState, queue_size=1)
        self.look_pub = rospy.Publisher(
            '/blender_api/set_gaze_target', Target, queue_size=1)
        self.turn_pub = rospy.Publisher(
            '/blender_api/set_face_target', Target, queue_size=1)
        exclude = '[%s]' % (re.escape(string.punctuation+u'你可以会能够吗，。？'))
        self.regex = re.compile(exclude, re.U)

    def _build_commands(self):
        self.motion_defs = []
        for motion_type, motions in self.config.items():
            for motion  in motions:
                motion['type'] = motion_type
                self.motion_defs.append(motion)

    def parse(self, msg):
        text = msg.utterance.decode('utf-8').lower()
        text = re.sub(r"\bcan you\b" , "", text)
        text = re.sub(r"\bdo\b" , "", text)
        text = re.sub(r"\bbe\b" , "", text)
        text = re.sub(r"\s+" , " ", text)
        text = self.regex.sub('', text)
        text = text.strip()
        for motion in self.motion_defs:
            if text in motion['pattern']:
                self.matched_motion = motion
                return True
        self.matched_motion = None
        return False

    def okay(self):
        lang = rospy.get_param('lang', None)
        if lang == 'en':
            self.chatbot_response_pub.publish(String("Okay."))
        if lang == 'zh':
            self.chatbot_response_pub.publish(String("好的。"))

    def execute(self):
        if self.matched_motion is not None:
            motion_type = self.matched_motion['type']
            if motion_type not in self.MSGTYPES:
                logger.error('Motion {} is not defined'.format(motion_type))
                return False
            msg = self.MSGTYPES[motion_type]()
            for name, value in self.matched_motion['params'].items():
                if name == 'duration':
                    value = rospy.Duration(value)
                setattr(msg, name, value)
            if motion_type == 'gesture':
                self.gesture_pub.publish(msg)
                rospy.loginfo('[#][COMMAND ACTION][GESTURE] {}'.format(msg))
            elif motion_type == 'emotion':
                self.emotion_pub.publish(msg)
                rospy.loginfo('[#][COMMAND ACTION][EMOTION] {}'.format(msg))
            elif motion_type == 'turn':
                self.turn_pub.publish(msg)
                rospy.loginfo('[#][COMMAND ACTION][LOOKAT] {}'.format(msg))
            elif motion_type == 'look':
                self.look_pub.publish(msg)
                rospy.loginfo('[#][COMMAND ACTION][GAZEAT] {}'.format(msg))
            self.okay()
            logger.info("Execute {} {}".format(motion_type,msg))
            return True

class MathCommand(BaseCommand):

    def __init__(self):
        super(MathCommand, self).__init__()
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
