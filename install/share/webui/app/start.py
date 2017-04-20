#!/usr/bin/python

#
# Server running with robot. Needs to be set up to run at startup for Robot PC.
# Requires for pololu_motors python package to be accessible.
# Requires mx_calib utility path (for dynamixel service)


import os
from flask import Flask, send_from_directory, redirect
from werkzeug.exceptions import NotFound
import reporter
import yaml
import logging
from optparse import OptionParser
from configs import *
from monitor import get_logs

json_encode = json.JSONEncoder().encode

app = Flask(__name__, static_folder='../public/')
rep = reporter.Reporter(os.path.dirname(os.path.abspath(__file__)) + '/checks.yaml')
app.add_url_rule('/monitor/logs/<loglevel>', None, get_logs, methods=['POST'])
# Global variables
config_root = None
robot_name = None

@app.route('/')
def send_index():
    return send_from_directory(app.static_folder, 'index.html')
@app.route('/monitor/start_status')
def send_start_status():
     return json_encode(rep.start_status(config_dir=config_root, robot_name=robot_name))

@app.route('/monitor/start_status/<action>')
def send_start_status_action(action):
    if action == 'start':
        return start_robot()
    if action == 'stop':
        return stop_robot()
    return json_encode(rep.start_status(config_dir=config_root, robot_name=robot_name))

def start_robot():
    if rep.start_robot(robot_name):
        return json_encode(rep.start_status(config_dir=config_root, robot_name=robot_name, started=2))
    else:
        return json_encode({'result': False})

def stop_robot():
    if rep.stop_robot(robot_name):
        return json_encode(rep.start_status(config_dir=config_root, robot_name=robot_name, started=0))
    else:
        return json_encode({'result': False})

@app.route('/public/<path:path>')
def send_js(path):
    return send_from_directory(app.static_folder, path)


@app.route('/robot_config.js')
def send_robot_js():
    try:
        return send_from_directory(os.path.join(config_root, robot_name), 'webstart.js')
    except NotFound:
        return ""

def read_yaml(file_name):
    f = open(file_name)
    data = yaml.load(f.read())
    f.close()
    return data

if __name__ == '__main__':
    # Option Parsing

    parser = OptionParser()
    parser.add_option("-r", "--robot", dest="robot", default="sophia", help="Robot Name", metavar="ROBOT_NAME")
    parser.add_option("-e", "--exec", dest="exe", default=None, help="Robot execution file", metavar="EXEC")
    parser.add_option("-s", "--stop", dest="stop", default=None, help="Robot stop file", metavar="EXEC_STOP")
    parser.add_option("-c", "--config", dest="config", default="", help="Robots config dir", metavar="CONFIG_DIR")
    parser.add_option("-p", "--port", dest="port", default=None, help="Port", metavar="PORT", type="int")
    parser.add_option("-m", "--mx", dest="mx", default=None, help="mx_calib path", metavar="PORT")

    (options, args) = parser.parse_args()
    config_root = options.config
    robot_name = options.robot
    rep.mx_tool = options.mx
    rep.exe = options.exe
    rep.exe_stop = options.stop
    # Run the server
    app.run(host='0.0.0.0', debug=False, use_reloader=False, port=options.port)
