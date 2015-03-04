#!/usr/bin/python3
import os

from flask import Flask, send_from_directory, request
import reporter
import json
import yaml
import math

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
    motors = json.loads(request.get_data().decode('utf8'))

    calibration = {}
    for motor in motors:
        if 'topic' in motor and 'motor_id' in motor:
            if not motor['topic'] in calibration:
                calibration[motor['topic']] = {}

            calibration[motor['topic']][motor['name']] = {
                'pololu_id': 0,
                'motor_id': motor['motor_id'],
                'init': radians_to_pulse(motor['default']),
                'min': radians_to_pulse(motor['min']),
                'max': radians_to_pulse(motor['min']),
                'reverse': False,
                'calibration': {
                    'min_pulse': radians_to_pulse(motor['min']),
                    'max_pulse': radians_to_pulse(motor['max']),
                    'min_angle': radians_to_degrees(motor['min']),
                    'max_angle': radians_to_degrees(motor['max']),
                }
            }

    for pololu in calibration.keys():
        file_name = '/catkin_ws/src/robots_config/' + robot_name + '/' + pololu + '_pololu.yaml'
        write_yaml(file_name, calibration[pololu])

    # write to motor config
    file_name = '/catkin_ws/src/robots_config/' + robot_name + '/motors.yaml'
    write_yaml(file_name, motors)

    return json_encode(True)


def write_yaml(file_name, data):
    # delete existing config
    try:
        os.remove(file_name)
    except OSError:
        pass

    f = open(file_name, 'w')
    f.write(yaml.dump(data))
    f.close()


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


@app.route('/expressions/<robot_name>', methods=['GET'])
def get_expressions(robot_name):
    f = open("/catkin_ws/src/robots_config/" + robot_name + "/expressions.yaml")
    expressions = yaml.load(f.read())
    f.close()

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


if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True, use_reloader=False)
