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
from subprocess import Popen
import logging
import time

json_encode = json.JSONEncoder().encode

app = Flask(__name__, static_folder='../public/')
rep = reporter.Reporter(os.path.dirname(os.path.abspath(__file__)) + '/checks.yaml')
config_root = os.path.join(os.path.dirname(os.path.abspath(__file__)),os.pardir,os.pardir,"robots_config")
logger = logging.getLogger('hr.ros_motors_webui.app')

@app.route('/')
def send_index():
    return send_from_directory(app.static_folder, 'index.html')


@app.route('/status')
def send_monitor_status():
    return json_encode(rep.report())

@app.route('/monitor/status')
def send_status():
    return json_encode(rep.system_status(config_dir=config_root))

@app.route('/motors/status/<robot_name>')
def get_motors_status(robot_name):
    if robot_name == 'undefined':
        return json_encode({})
    motors = read_yaml(os.path.join(config_root,robot_name, 'motors_settings.yaml'))
    motors = rep.motor_states(motors, robot_name)
    return json_encode({'motors': motors})

@app.route('/motors/get/<robot_name>')
def get_motors(robot_name):
    motors = read_yaml(os.path.join(config_root,robot_name, 'motors_settings.yaml'))
    return json_encode({'motors': motors})

@app.route('/monitor/logs/<loglevel>/<timestamp>')
def get_logs(loglevel, timestamp):
    """
    Collect all the warnings, errors and fatals from the ros log files.
    If there is no roscore process running, then it displays the logs
    from the last run.
    """

    logger.info('get logs: log level {}'.format(loglevel))
    from roslaunch.roslaunch_logs import get_run_id
    import rospkg
    import glob
    import re
    log_root = rospkg.get_log_dir()
    run_id = get_run_id()
    roscore_running = True
    if not run_id:
        roscore_running = False
        subdirs = [os.path.join(log_root, d) for d in os.listdir(log_root)
                    if os.path.isdir(os.path.join(log_root, d))]
        if subdirs:
            run_id = max(subdirs, key=os.path.getmtime)
        else:
            run_id = ''

    # some extra log files that not created by roslaunch
    extra_log_files = [os.path.join(log_root, name) for name in [
        'ros_motors_webui.log', 'sophia_Eva_Behavior.log', 'blender_api.log']]
    extra_log_files = [f for f in extra_log_files if os.path.isfile(f)]

    log_dir = os.path.join(log_root, run_id)
    log_files = glob.glob(os.path.join(log_dir, '*.log'))
    log_files += extra_log_files

    # ignore stdout log files
    log_files = [log_file for log_file in log_files if ('stdout' not in log_file) and (not os.path.basename(log_file).startswith('roslaunch'))]
    log_files = sorted(log_files, key=lambda f: os.path.basename(f))
    logger.info('get log files: {}'.format('\n'.join(log_files)))
    logs = []

    # log format [%(name)s][%(levelname)s] %(asctime)s: %(message)s
    pattern = r'\[(?P<name>\S+)\]\[(?P<levelname>\S+)\] (?P<asctime>\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2},\d{3}): (?P<message>.*)'
    p = re.compile(pattern)
    loglevels = {
        'debug': 0,
        'info': 1,
        'warn': 2,
        'warning': 2,
        'error': 3,
        'fatal': 4,
    }

    def get_log(log_file, loglevel, truncate_th=100):
        _log = []
        truncate = False
        loglevel = loglevels[loglevel.lower()]
        message_length = 120
        with open(log_file) as f:
            logrecord = None
            for line in f.read().splitlines():
                m = p.match(line)
                if m:
                    if logrecord:
                        if loglevels[logrecord['levelname'].lower()] >= loglevel:
                            _log.append(logrecord)
                        logrecord = None
                    if len(_log) >= truncate_th:
                        truncate = True
                        break
                    name, levelname, asctime, message = map(
                        m.group, ['name', 'levelname', 'asctime', 'message'])
                    extra = []
                    if len(message) > message_length:
                        extra.append(message[message_length:])
                        message = message[:message_length] + ' ...'
                    logrecord = {
                        'name': name,
                        'levelname': levelname,
                        'asctime': asctime,
                        'message': message,
                        'extra': extra,
                    }
                # Append message that doesn't match the log format to the
                # previous matched log record
                elif logrecord:
                    logrecord['extra'].append(line)

        return truncate, _log

    def isint(i):
        try:
            int(i)
        except:
            return False
        return True

    def parse_node_name(log_file):
        base = os.path.splitext(os.path.basename(log_file))[0]
        tokens = base.split('-')
        if 'roslaunch' in base:
            return base
        try:
            idx = map(isint, tokens).index(True)
            node = '/'.join(tokens[:idx])
        except:
            node = '/'.join(tokens)
        return node

    for log_file in log_files:
        node = parse_node_name(log_file)
        truncate, log = get_log(log_file, loglevel)
        if log:
            logs.append({
                'node': node,
                'log': log,
                'truncate': truncate,
                'log_file': log_file,
                })
    result = {
        'logs': logs,
        'timestamp': time.time()
    }
    return json_encode(result)

def reload_configs(motors,config_dir, robot_name):
    configs = Configs()
    configs.parseMotors(motors)
    if len(configs.dynamixel) > 0:
        file_name = os.path.join(config_dir,"dynamixel.yaml")
        write_yaml(file_name,configs.dynamixel)
        load_params(file_name, "/{}/safe".format(robot_name))
    if len(configs.motors) > 0:
        file_name = os.path.join(config_dir,"motors.yaml")
        write_yaml(file_name,{'motors': configs.motors})
        load_params(file_name, "/{}".format(robot_name))
    if len(configs.pololu) > 0:
        for board, config in configs.pololu.iteritems():
            file_name = os.path.join(config_dir,board + ".yaml")
            write_yaml(file_name,config)
            kill_node("/{}/pololu_{}".format(robot_name, board))
    kill_node("/{}/pau2motors".format(robot_name))
    return configs


@app.route('/motors/update/<robot_name>', methods=['POST'])
def update_motors(robot_name):
    motors = json.loads(request.get_data())
    # get rid of false data
    motors = [m for m in motors if 'hardware' in m.keys()]
    # write to motor config
    try:
        file_name = os.path.join(config_root,robot_name, 'motors_settings.yaml')
        reload_configs(motors,os.path.join(config_root,robot_name), robot_name)
        write_yaml(file_name, motors)
    except Exception as e:
        return json_encode({'error': str(e)})
    return json_encode({'error': False})


@app.route('/expressions/<robot_name>', methods=['GET'])
def get_expressions(robot_name):
    expressions = read_yaml(os.path.join(config_root, robot_name, "expressions.yaml"))
    return json_encode(expressions)


@app.route('/expressions/update/<robot_name>', methods=['POST'])
def update_expressions(robot_name):
    data = json.loads(request.get_data().decode('utf8'))
    filename = os.path.join(config_root, robot_name, "expressions.yaml")

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
    file_name = os.path.join(config_root, robot_name, "animations.yaml")

    write_yaml(file_name, data)
    return json_encode(True)


@app.route('/performances/get/<robot_name>', methods=['GET'])
def get_performances(robot_name):
    performances = read_yaml(os.path.join(config_root,robot_name, 'performances.yaml'))
    return json_encode(performances)

@app.route('/performances/update/<robot_name>', methods=['POST'])
def update_performances(robot_name):
    performances = json.loads(request.get_data())

    try:
        file_name = os.path.join(config_root, robot_name, 'performances.yaml')
        write_yaml(file_name, performances)
    except Exception as e:
        return json_encode({'error': str(e)})

    return json_encode({'error': False})


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


def radians_t0o_degrees(rad):
    return round(rad * 180 / math.pi)


def kill_node(node):
    Popen("rosnode kill " + node, shell=True)

def load_params(param_file, namespace):
    Popen("rosparam load " + param_file +" " + namespace, shell=True)


if __name__ == '__main__':
    #from rosgraph.roslogging import configure_logging
    #configure_logging(logger.name, filename='ros_motors_webui.log')

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
