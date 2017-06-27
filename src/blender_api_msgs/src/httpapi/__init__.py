import os
import logging
import threading
from flask import Flask, Response, request
import json

logger = logging.getLogger('hr.blender_api.httpapi')
handler = logging.FileHandler(os.path.expanduser('~/.hr/blender_httpapi.log'))
logger.addHandler(handler)
logger.setLevel(logging.INFO)
logger.info("Start")

api = None
app = Flask(__name__)
json_encode = json.JSONEncoder().encode

def build(rigapi):
    global api
    api = rigapi
    server = threading.Timer(0, app.run,
        kwargs={'host':'0.0.0.0', 'debug':False, 'port':9001})
    server.deamon = True
    server.start()
    return Nothing()

@app.route('/getAPIVersion', methods=['GET'])
def _getApiVersion():
    version = None
    try:
        version = api.getAPIVersion()
        logger.info(version)
    except Exception as ex:
        logger.error(ex)
    return Response(json_encode({'version': version}),
                mimetype="application/json")

@app.route('/setGesture', methods=['GET'])
def _setGesture():
    try:
        if api.pauAnimationMode & api.PAU_ACTIVE > 0:
            return
        data = request.args
        name = data.get('name')
        repeat = int(data.get('repeat'))
        speed = float(data.get('speed'))
        magnitude = float(data.get('magnitude'))
        api.setGesture(name, repeat, speed, magnitude)
        return Response(json_encode({'success': True}),
                    mimetype="application/json")
    except Exception as ex:
        logger.error('Unknown gesture: {}'.format(ex))
        return Response(json_encode({'success': False}),
                    mimetype="application/json")

class Nothing():

    def init(self):
        logger.info("init")
        return True

    def poll(self):
        logger.info("poll")

    def push(self):
        logger.info("push")
        return True

    def drop(self):
        logger.info("drop")
        return True
