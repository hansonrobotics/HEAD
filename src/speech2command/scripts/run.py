#!/usr/bin/env python2.7
import os
import rospy
import logging

from chatbot.msg import ChatMessage
from speech2command.commands import ForwardCommand
from dynamic_reconfigure.server import Server
from speech2command.cfg import Speech2CommandConfig

logger = logging.getLogger('hr.speech2command.speech2command')

class Speech2Command(object):

    def __init__(self, config):
        rospy.Subscriber('speech', ChatMessage, self.handle_speech)
        self.command_chain = [
            ForwardCommand('chatbot_speech')]

    def handle_speech(self, msg):
        for command in self.command_chain:
            try:
                if command.active and command.parse(msg):
                    command.execute()
                    break
            except Exception as ex:
                logger.error("Handle speech message error: {}".format(ex))
                continue

    def reconfig(self, config, level):
        for command in self.command_chain:
            if command.type == 'wholeshow':
                command.active = config.activate_wholeshow
        return config


if __name__ == '__main__':
    rospy.init_node('speech2command')
    from rospkg import RosPack                                                    
    rp = RosPack()                                                                
    node = Speech2Command(
        os.path.join(
            rp.get_path('speech2command'), 'motion_command_config.yaml'))
    Server(Speech2CommandConfig, node.reconfig)
    rospy.spin()

