#!/usr/bin/env python2.7
import rospy
import logging
from chatbot.msg import ChatMessage
from commands import command_chain

logger = logging.getLogger('hr.speech2command.speech2command')

class Speech2Command(object):

    def __init__(self):
        rospy.Subscriber('speech', ChatMessage, self.handle_speech)

    def handle_speech(self, msg):
        for command in command_chain:
            if command.parse(msg):
                command.execute()
                logger.info("Execute command {}".format(command))
                break

if __name__ == '__main__':
    rospy.init_node('speech2command')
    Speech2Command()
    rospy.spin()

