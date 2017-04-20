#!/usr/bin/env python2.7
import os
import rospy
import logging
from chatbot.msg import ChatMessage
from speech2command.commands import MotionCommand, MathCommand, ForwardCommand, WholeShowCommand

logger = logging.getLogger('hr.speech2command.speech2command')
CWD = os.path.dirname(os.path.abspath(__file__))

class Speech2Command(object):

    def __init__(self, config):
        rospy.Subscriber('speech', ChatMessage, self.handle_speech)
        self.command_chain = [
            WholeShowCommand(),
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
    from rospkg import RosPack                                                    
    rp = RosPack()                                                                
    Speech2Command(
        os.path.join(
            rp.get_path('speech2command'), 'motion_command_config.yaml'))
    rospy.spin()

