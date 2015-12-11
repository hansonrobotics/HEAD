#!/usr/bin/env python2.7
import rospy
import logging
from chatbot.msg import ChatMessage
from commands import MotionCommand, MathCommand, ForwardCommand

logger = logging.getLogger('hr.speech2command.speech2command')

class Speech2Command(object):

    def __init__(self):
        rospy.Subscriber('speech', ChatMessage, self.handle_speech)
        self.command_chain = [
            MotionCommand('motion_command_config.yaml'), MathCommand(),
            ForwardCommand('chatbot_speech')]

    def handle_speech(self, msg):
        for command in self.command_chain:
            try:
                if command.parse(msg):
                    command.execute()
                    break
            except Exception as ex:
                logger.error("Handle speech message error: {}".format(ex))
                continue

if __name__ == '__main__':
    rospy.init_node('speech2command')
    Speech2Command()
    rospy.spin()

