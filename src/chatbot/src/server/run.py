#!/usr/bin/env python2.7

import os
import logging
from auth import requires_auth

LOG_CONFIG_FILE = os.environ.get('ROS_PYTHON_LOG_CONFIG_FILE', None)
if LOG_CONFIG_FILE is not None:
    import logging.config
    import rosgraph
    if 'ROS_LOG_DIR' not in os.environ:
        os.environ['ROS_LOG_DIR'] = os.path.expanduser('~/.hr')
    if 'ROS_LOG_FILENAME' not in os.environ:
        os.environ['ROS_LOG_FILENAME'] = os.path.join(
            os.environ['ROS_LOG_DIR'], 'chatbot_server.log')
    logging.config.fileConfig(LOG_CONFIG_FILE)
else:
    logging.basicConfig()
    logging.getLogger().setLevel(logging.INFO)

import sys
from flask import Flask, request, Response
import json
from chatbot import ask, list_character

json_encode = json.JSONEncoder().encode
app = Flask(__name__)

@app.route('/', methods=['POST'])
@requires_auth
def chat():
    data = request.get_json()
    botname = data['botname']
    question = data['question']
    session = data['session']
    response, ret = ask(botname, question, session)
    return Response(json_encode({'ret': ret, 'response': response}),
        mimetype="application/json")

@app.route('/chatbots', methods=['GET'])
def list_chatbot():
    return Response(json_encode({'ret': 0, 'response': list_character()}),
        mimetype="application/json")

if __name__ == '__main__':
    if len(sys.argv) > 1:
        port = int(sys.argv[1])
    else:
        port = 8001
    app.run(host='0.0.0.0', debug=False, use_reloader=False, port=port)
