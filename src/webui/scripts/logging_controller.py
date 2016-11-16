#!/usr/bin/env python

import rospy
import os
import time
import logging
import std_msgs.msg as msg
import yaml
import json
from rospkg import RosPack

rp = RosPack()

logger = logging.getLogger('hr.webui.bug_controller')
data_dir = os.path.join(rp.get_path('webui'), 'data')


class LoggingController:
    def __init__(self):
        self.subscriber = rospy.Subscriber('/webui/log/chat', msg.String, self.chat_logger)
        self.subscriber = rospy.Subscriber('/webui/log/bugs', msg.String, self.bug_logger)
        rospy.init_node('bug_controller')
        rospy.spin()

    def bug_logger(self, request):
        self.append(os.path.join(data_dir, 'bug_reports.yaml'), request.data)

    def chat_logger(self, request):
        self.append(os.path.join(data_dir, 'chat', time.strftime("%Y/%m/%d") + '.yaml'), json.loads(request.data))

    @staticmethod
    def append(file_name, data):
        LoggingController.write_yaml(file_name, 'a', data)

    @staticmethod
    def write(file_name, data):
        LoggingController.write_yaml(file_name, 'w', data)

    @staticmethod
    def write_yaml(file_name, mode, data):
        if not os.path.exists(os.path.dirname(file_name)):
            os.makedirs(os.path.dirname(file_name))

        with open(file_name, mode) as f:
            f.write(yaml.safe_dump([{'data': data, 'time': time.strftime("%Y/%m/%d %H:%M:%S")}], encoding='utf-8',
                                   allow_unicode=True))

    @staticmethod
    def read(file_name):
        if os.path.isfile(file_name):
            with open(file_name) as f:
                return yaml.load(f.read())
        else:
            return ''


if __name__ == '__main__':
    LoggingController()
