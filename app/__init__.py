#!/usr/bin/python
import os

from flask import Flask, send_from_directory, request
import reporter
import json
import yaml
import math
import os.path

json_encode = json.JSONEncoder().encode

app = Flask(__name__, static_folder='../public/')
rep = reporter.Reporter(os.path.dirname(os.path.abspath(__file__)) + '/checks.yaml')


@app.route('/')
def send_index():
    return send_from_directory(app.static_folder, 'index.html')


@app.route('/status')
def send_status():
    return json_encode(rep.report())


@app.route('/motors/get/<robot_name>')
def get_motors(robot_name):
    config_root = "/catkin_ws/src/robots_config/" + robot_name + "/"
    motors = read_yaml(config_root + "motors.yaml")
    calibration = {}

    for motor in motors:
        # skip motors without motor_id or topic
        if 'topic' not in motor or 'motor_id' not in motor or 'name' not in motor:
            continue

        # load calibration file if not loaded yet
        if not motor['topic'] in calibration:
            calibration_config = config_root + motor['topic'] + "_pololu.yaml"

            if os.path.isfile(calibration_config):
                calibration[motor['topic']] = read_yaml(calibration_config)

        motor['calibration'] = calibration[motor['topic']][motor['name']]

    return json_encode(motors)


@app.route('/motors/update/<robot_name>', methods=['POST'])
def update_motors(robot_name):
    motors = json.loads(request.get_data().decode('utf8'))

    calibration = {}
    for motor in motors:
        if 'topic' in motor and 'motor_id' in motor:
            if not motor['topic'] in calibration:
                calibration[motor['topic']] = {}

            if 'calibration' in motor:
                calibration[motor['topic']][motor['name']] = motor['calibration']
                del motor['calibration']
            else:
                calibration[motor['topic']][motor['name']] = {
                    'pololu_id': 0,
                    'motor_id': motor['motor_id'],
                    'init': radians_to_pulse(motor['default']),
                    'min': radians_to_pulse(motor['min']),
                    'max': radians_to_pulse(motor['max']),
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
    f.write(yaml.dump(data))
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
    app.run(host='0.0.0.0', debug=True, use_reloader=True)
