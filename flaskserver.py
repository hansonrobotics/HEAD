#!/usr/bin/python3
import os

from flask import Flask, send_from_directory, request
import json
import reporter
import yaml

json_encode = json.JSONEncoder().encode

app = Flask(__name__)
rep = reporter.Reporter('checks.yaml')

@app.route('/system/status')
def send_status():
    return json_encode(rep.report())

@app.route('/')
def send_index():
    return send_public('index.html')

@app.route('/<path:filename>')
def send_public(filename):
    return send_from_directory('public', filename)

@app.route('/<robot_name>/motors/update', methods=['POST'])
def update_motor_config(robot_name):
    data = json.loads(request.get_data().decode('utf8'))
    filename = '/catkin_ws/src/robots_config/' + robot_name + '/motors.yaml'

    # delete existing config
    try:
        os.remove(filename)
    except OSError:
        pass

    # write new config
    f = open(filename, 'w')
    f.write(yaml.dump(data))
    f.close()

    # return True
    return json_encode(True)

if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True, use_reloader=False)
