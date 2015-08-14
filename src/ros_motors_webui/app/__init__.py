#!/usr/bin/python
import os
import os.path

from flask import Flask, send_from_directory, request
import reporter
import json
import yaml
import math
import os.path
from optparse import OptionParser
from configs import *

json_encode = json.JSONEncoder().encode

app = Flask(__name__, static_folder='../public/')
rep = reporter.Reporter(os.path.dirname(os.path.abspath(__file__)) + '/checks.yaml')
config_root = os.path.join(os.path.dirname(os.path.abspath(__file__)),os.pardir,os.pardir,"robots_config")

@app.route('/')
def send_index():
    return send_from_directory(app.static_folder, 'index.html')


@app.route('/status')
def send_status():
    return json_encode(rep.report())


@app.route('/motors/get/<robot_name>')
def get_motors(robot_name):
    motors = read_yaml(os.path.join(config_root,robot_name, 'motors_settings.yaml'))
    return json_encode({'motors': motors})

def set_configs(motors,config_dir):
    configs = Configs()
    configs.parseMotors(motors)
    if len(configs.dynamixel) > 0:
        file_name = os.path.join(config_dir,"dynamixel.yaml")
        write_yaml(file_name,configs.dynamixel)
    if len(configs.motors) > 0:
        file_name = os.path.join(config_dir,"motors.yaml")
        write_yaml(file_name,{'motors': configs.motors})
    if len(configs.pololu) > 0:
        for board, config in configs.pololu.iteritems():
            file_name = os.path.join(config_dir,board + ".yaml")
            write_yaml(file_name,config)
    return configs


@app.route('/motors/update/<robot_name>', methods=['POST'])
def update_motors(robot_name):
    motors = json.loads(request.get_data())

    # write to motor config
    try:
        file_name = os.path.join(config_root,robot_name, 'motors_settings.yaml')
        set_configs(motors,os.path.join(config_root,robot_name))
        write_yaml(file_name, motors)
    except Exception as e:
        return json_encode({'error': str(e)})
    return json_encode({'error': False})


@app.route('/expressions/<robot_name>', methods=['GET'])
def get_expressions(robot_name):
    expressions = read_yaml("/catkin_ws/src/robots_config/" + robot_name + "/expressions.yaml")
    return json_encode(expressions)


@app.route('/expressions/update/<robot_name>', methods=['POST'])
def update_expressions(robot_name):
    data = json.loads(request.get_data().decode('utf8'))
    filename = "/catkin_ws/src/robots_config/" + robot_name + "/expressions.yaml"

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

@app.route('/animations/update/<robot_name>', methods=['POST'])
def update_animations(robot_name):
    data = json.loads(request.get_data().decode('utf8'))
    file_name = "/catkin_ws/src/robots_config/" + robot_name + "/animations.yaml"

    write_yaml(file_name, data)
    return json_encode(True)


def write_yaml(file_name, data):
    # delete existing config
    try:
        os.remove(file_name)
    except OSError:
        pass

    f = open(file_name, 'w')
    f.write(yaml.safe_dump(data, encoding='utf-8', allow_unicode=True))
    f.close()


def read_yaml(file_name):
    f = open(file_name)
    data = yaml.load(f.read())
    f.close()

    return data


def radians_to_pulse(rad):
    # equals -PI / 2
    pulse_min = 820

    # equals PI / 2
    pulse_max = 2175

    one_rad_pulse = pulse_max - pulse_min
    zero_pulse = pulse_min + one_rad_pulse / 2
    return round(zero_pulse + one_rad_pulse * rad)


def radians_to_degrees(rad):
    return round(rad * 180 / math.pi)

if __name__ == '__main__':
    @app.route('/public/<path:path>')
    def send_js(path):
        return send_from_directory(app.static_folder, path)

    parser = OptionParser()
    parser.add_option("-s", action="store_true", dest="ssl", default=False, help="Use SSL")
    parser.add_option("-c", "--cert", dest="cert", default="", help="SSL Certificate", metavar="CERT_FILE")
    parser.add_option("-k", "--key", dest="key", default="", help="SSL Key", metavar="KEY_FILE")
    parser.add_option("-p", "--port", dest="port", default=None, help="Port", metavar="KEY_FILE", type="int")
    (options, args) = parser.parse_args()
    if options.ssl:
        if not options.cert:
            parser.error("Certificate must be specified for SSL")
        if not os.path.isfile(options.cert):
            parser.error("Certificate file does not exists")
        if not options.key:
            parser.error("Key must be specified for SSL")
        if not os.path.isfile(options.key):
            parser.error("Key file does not exists")
        context = (options.cert, options.key)
        app.run(host='0.0.0.0', debug=True, use_reloader=True, ssl_context=context, port=options.port)
    else:
        app.run(host='0.0.0.0', debug=True, use_reloader=True, port=options.port)
