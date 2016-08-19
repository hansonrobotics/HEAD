#!/usr/bin/env python

import rospy
import os
import logging
import json
from rospkg import RosPack
import webui.srv as srv
import requests
import urllib

rp = RosPack()

logger = logging.getLogger('hr.webui.bug_controller')
data_dir = os.path.join(rp.get_path('webui'), 'data')


class ChatbotController:
    def __init__(self):
        self.auth_token = 'AAAAB3NzaC'
        rospy.init_node('chatnot_controller')
        rospy.Service('~bot_names', srv.Json, self.bot_names_callback)
        rospy.spin()

    def bot_names_callback(self, request):
        return self.get_json_response('bot_names')

    def get_json_response(self, path, request=None):
        params = {'Auth': self.auth_token}
        if isinstance(request, dict):
            params.update(request)
        r = requests.get('http://127.0.0.1:8001/v1.1/' + path + '?' + urllib.urlencode(params))
        response = '{}'
        success = False

        if r.status_code == 200:
            response = r.json()
            success = True

        return srv.JsonResponse(success=success, response=json.dumps(response))


if __name__ == '__main__':
    ChatbotController()
