#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

import os
import json

import sys
CWD = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(CWD, '../src'))

from flask import Flask, request, Response
from vision.face_detector import detect_face

json_encode = json.JSONEncoder().encode
app = Flask(__name__)
VERSION = 'v1.0'
ROOT = '/{}'.format(VERSION)


@app.route(ROOT + '/detect_face', methods=['POST'])
def _detect_face():
    image = request.data
    args = request.args.to_dict()
    for k, v in args.iteritems():
        if v.lower() == 'false':
            args[k] = False
        elif v.lower() == 'true':
            args[k] = True
    response = detect_face(image, **args)
    return Response(json_encode({'response': response}),
                    mimetype="application/json")


if __name__ == '__main__':
    if len(sys.argv) > 1:
        port = int(sys.argv[1])
    else:
        port = 9001
    app.run(host='0.0.0.0', debug=False, use_reloader=False, port=port)
