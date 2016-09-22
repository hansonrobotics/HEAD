#!/usr/bin/python
import os
import fnmatch
from flask import Flask, send_from_directory, request, Response, g
from werkzeug.utils import secure_filename
import reporter
import yaml
import logging
from optparse import OptionParser
from configs import *
from subprocess import Popen
import datetime
from monitor import get_logs
from rospkg import RosPack
from pi_face_tracker.msg import Faces, Face
import copy

rp = RosPack()
import rospy

json_encode = json.JSONEncoder().encode

app = Flask(__name__, static_folder='../public/')
app.config['CHAT_AUDIO_DIR'] = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, 'chat_audio')
rep = reporter.Reporter(os.path.dirname(os.path.abspath(__file__)) + '/checks.yaml')
config_root = rp.get_path('robots_config')
performance_dir = os.path.join(rp.get_path('performances'), 'robots')
shared_performances_folder = 'shared'

app.add_url_rule('/monitor/logs/<loglevel>', None, get_logs, methods=['POST'])


def get_pub():
    db = getattr(g, '_database', None)
    if db is None:
        db = g._database = connect_to_database()
    return db


@app.route('/')
def send_index():
    return send_from_directory(app.static_folder, 'index.html')


@app.route('/robot_config_schema')
def get_robot_config_schema():
    config = read_yaml(os.path.join(config_root, 'config_schema.yaml'))
    return json_encode(config)


@app.route('/robot_config/<robot_name>')
def get_robot_config(robot_name):
    if robot_name == 'undefined':
        return json_encode({})

    config = read_yaml(os.path.join(config_root, robot_name, 'config.yaml'))
    return json_encode(config)


@app.route('/robot_config/<robot_name>', methods=['POST'])
def set_robot_config(robot_name):
    motors = json.loads(request.get_data())
    try:
        file_name = os.path.join(config_root, robot_name, 'config.yaml')
        # write_yaml(file_name, motors)
    except Exception as e:
        return json_encode({'error': str(e)})

    return json_encode(read_yaml(os.path.join(config_root, robot_name, 'config.yaml')))


@app.route('/status')
def send_monitor_status():
    return json_encode(rep.report())


@app.route('/chat_audio', methods=['POST'])
def chat_audio():
    audio = request.files['audio']
    ts = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    filename = secure_filename(ts + '.wav')
    filename = os.path.join(app.config['CHAT_AUDIO_DIR'], filename)
    audio.save(filename)
    return Response(json_encode({'success': True, 'filename': filename}), mimetype="application/json")


@app.route('/monitor/status')
def send_status():
    return json_encode(rep.system_status(config_dir=config_root))


@app.route('/motors/status/<robot_name>')
def get_motors_status(robot_name):
    if robot_name == 'undefined':
        return json_encode({})
    motors = read_yaml(os.path.join(config_root, robot_name, 'motors_settings.yaml'))
    motors = rep.motor_states(motors, robot_name)
    return json_encode({'motors': motors})


@app.route('/motors/get/<robot_name>')
def get_motors(robot_name):
    motors = read_yaml(os.path.join(config_root, robot_name, 'motors_settings.yaml'))
    return json_encode({'motors': motors})


def reload_configs(motors, config_dir, robot_name):
    configs = Configs()
    configs.parseMotors(motors)
    if len(configs.dynamixel) > 0:
        file_name = os.path.join(config_dir, "dynamixel.yaml")
        write_yaml(file_name, configs.dynamixel)
        load_params(file_name, "/{}/safe".format(robot_name))
    if len(configs.motors) > 0:
        file_name = os.path.join(config_dir, "motors.yaml")
        write_yaml(file_name, {'motors': configs.motors})
        load_params(file_name, "/{}".format(robot_name))
    if len(configs.pololu) > 0:
        for board, config in configs.pololu.iteritems():
            file_name = os.path.join(config_dir, board + ".yaml")
            write_yaml(file_name, config)
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
        file_name = os.path.join(config_root, robot_name, 'motors_settings.yaml')
        reload_configs(motors, os.path.join(config_root, robot_name), robot_name)
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


@app.route('/attention_regions/<robot_name>', methods=['GET'])
def get_attention_regions(robot_name):
    areas = []
    path = os.path.join(config_root, robot_name, 'attention_regions.yaml')
    if os.path.exists(path):
        areas = read_yaml(path)
        areas = areas['attention_regions']
    return json_encode(areas)


@app.route('/attention_regions/<robot_name>', methods=['POST'])
def update_attention_regions(robot_name):
    path = os.path.join(config_root, robot_name, 'attention_regions.yaml')
    regions = json.loads(request.get_data())
    regions = {'attention_regions': regions}
    write_yaml(path, regions)
    # Reload regions so can be edited live.
    try:
        load_params(path, robot_name)
    except:
        pass
    return json_encode(regions)


def get_performance(id, robot_name):
    root = os.path.join(config_root, robot_name, 'performances')
    filename = os.path.join(root, id + '.yaml')
    if os.path.isfile(filename):
        performance = read_yaml(filename)
        relative = filename.split('/')[len(root.split('/')):]
        performance['name'] = relative[-1] = relative[-1].split('.')[0]
        performance['id'] = os.path.join(*relative)
        relative_path = relative[:-1]
        performance['path'] = os.path.join(*relative_path) if relative_path else ''
        del performance['nodes']
        return performance
    else:
        return {}


@app.route('/performances/<robot_name>', methods=['GET'])
def get_performances(robot_name):
    performances = []
    robot_root = os.path.join(config_root, robot_name, 'performances')
    # Common performances for robots
    common_root = os.path.join(config_root, 'common', 'performances')
    ids = []
    for root in [robot_root, common_root]:
        if root == common_root:
            robot_name = 'common'
        for path, dirnames, filenames in os.walk(root):
            for name in fnmatch.filter(filenames, '*.yaml'):
                filename = os.path.join(path, name)
                id = os.path.join(*filename.split('.')[0].split('/')[len(root.split('/')):])
                # Prevent duplicate ids
                if id not in ids:
                    performances.append(get_performance(id, robot_name))
                    ids.append(id)
    return json_encode(performances)


@app.route('/performances/<robot_name>/<path:id>', methods=['PUT', 'POST'])
def update_performances(robot_name, id):
    performance = json.loads(request.get_data())
    if id.startswith(shared_performances_folder):
        robot_name = 'common'
    root = os.path.join(config_root, robot_name, 'performances')
    if not os.path.exists(root):
        os.makedirs(root)

    try:
        filename = os.path.join(root, id + '.yaml')
        current = {}

        if 'previous_id' in performance:
            previous = os.path.join(root, performance['previous_id'] + '.yaml')
            if os.path.isfile(previous):
                current = read_yaml(previous)
                os.remove(previous)
            del performance['previous_id']
        elif os.path.isfile(filename):
            current = read_yaml(filename)

        for key in ['id', 'path']:
            if key in performance: del performance[key]

        if 'ignore_nodes' in performance and performance['ignore_nodes'] and 'nodes' in current:
            performance['nodes'] = current['nodes']
            del performance['ignore_nodes']

        content = copy.deepcopy(performance)
        content.pop('id', None)
        content.pop('path', None)

        write_yaml(filename, content)
    except Exception as e:
        return json_encode({'error': str(e)})

    return json_encode(performance)


@app.route('/performances/keywords/<robot_name>', methods=['GET'], defaults={'path': ''})
@app.route('/performances/keywords/<robot_name>/<path:path>', methods=['GET'])
def get_performance_keywords(robot_name, path):
    if path.startswith(shared_performances_folder):
        robot_name = 'common'
    filename = os.path.join(config_root, robot_name, 'performances', path, '.properties')
    keywords = {'keywords': []}
    if os.path.isfile(filename):
        keywords = read_yaml(filename)
    return json_encode(keywords)


@app.route('/performances/keywords/<robot_name>', methods=['POST'], defaults={'path': ''})
@app.route('/performances/keywords/<robot_name>/<path:path>', methods=['POST'])
def update_performance_keywords(robot_name, path):
    if path.startswith(shared_performances_folder):
        robot_name = 'common'
    filename = os.path.join(config_root, robot_name, 'performances', path, '.properties')
    data = request.get_data()
    data = json.loads(data)
    keywords = []
    if 'keywords' in data and isinstance(data['keywords'], list):
        keywords = data['keywords']
        for i, keyword in enumerate(keywords):
            keywords[i] = keyword.strip()
        keywords = filter(bool, keywords)

    write_yaml(filename, {'keywords': keywords})
    return json_encode(True)


@app.route('/performances/<robot_name>/<path:id>', methods=['DELETE'])
def delete_performances(robot_name, id):
    if id.startswith(shared_performances_folder):
        robot_name = 'common'
    performance = os.path.join(config_root, robot_name, 'performances', id + '.yaml')

    if os.path.isfile(performance):
        os.remove(performance)
        return json_encode(True)
    else:
        return json_encode(False)


@app.route('/run_performance', methods=['POST'])
def start_performance():
    performance = request.get_data()
    js = json.loads(performance)
    run_performance(js["key"])
    return json_encode({'result': True})


@app.route('/slide_performance/<performance>', methods=['GET'])
def slide_performance(performance):
    run_performance(performance)
    return json_encode({'result': True})


@app.route('/lookat', methods=['POST'])
def look_at():
    performance = request.get_data()
    js = json.loads(performance)
    # js = {'x':1,'y':0.3,'z':-0.5}
    f = Face()
    f.id = 1
    f.point.x = js["x"]
    f.point.y = -js["y"]
    f.point.z = js["z"]
    f.attention = 0.99
    facePub.publish(Faces([f]))
    return json_encode({'result': True})


@app.route('/realsense', methods=['POST'])
def realsense():
    stream = request.get_json()
    # iterate over each element in json
    f = Faces()
    faces = []
    if stream != None:
        for i in stream:
            if i is not None:
                if 'people' in i and stream[i] is not None:
                    for j in range(0, len(stream[i])):
                        p = Face()
                        p.id = stream[i][j]['ID']
                        p.point.x = (stream[i][j]['z']) / 1000.0
                        p.point.y = -(stream[i][j]['x']) / 1000.0  # Center of the face
                        p.point.z = (stream[i][j]['y']) / 1000.0

                        # p.pose.x = stream[i][j]['rx']
                        # p.pose.y = stream[i][j]['ry']
                        # p.pose.z = stream[i][j]['rz']
                        # p.pose.w = stream[i][j]['rw']

                        p.attention = stream[i][j][
                            'confidence']  # This is an indication that the depth image is near or far a way. Low values indicate that it's not a good value.

                        # Now let's get the emotion related data out of the elements.

                        # if stream[i][j]['expression'] is not None:
                        #     for expressions in j['expression']: # Now this equivalent makes sure that there is alway the expression being processed in the realsense demo application
                        #             p.emotions.append(str(expressions))
                        #             p.emotion_values.append(stream[i][j]['expression'][expressions])
                        faces.append(p)

    if len(faces) > 0:
        f.faces = faces  # Check if it's not empty before publishing it.
        facePub.publish(f)
    return Response(json_encode({'success': True}), mimetype="application/json")


@app.route('/robot_config.js')
def send_robot_js():
    # Returns mode as normal
    return "define(function (){return {mode:'normal'}});"


def write_yaml(filename, data):
    # delete existing config
    try:
        os.remove(filename)
    except OSError:
        pass

    dir = os.path.dirname(filename)
    if not os.path.exists(dir):
        os.makedirs(dir)

    f = open(filename, 'w')
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
    Popen("rosparam load " + param_file + " " + namespace, shell=True)


def run_performance(performance):
    Popen("rosservice call /performances/run_by_name \"idf/{}\"".format(performance), shell=True)


if __name__ == '__main__':
    logging.getLogger().setLevel(logging.INFO)
    logging.basicConfig()


    @app.route('/public/<path:path>')
    def send_js(path):
        return send_from_directory(app.static_folder, path)


    parser = OptionParser()
    parser.add_option("-s", action="store_true", dest="ssl", default=False, help="Use SSL")
    parser.add_option("-c", "--cert", dest="cert", default="", help="SSL Certificate", metavar="CERT_FILE")
    parser.add_option("-k", "--key", dest="key", default="", help="SSL Key", metavar="KEY_FILE")
    parser.add_option("-p", "--port", dest="port", default=None, help="Port", metavar="PORT", type="int")
    parser.add_option("-a", "--audio", dest="audio", default=None, help="Destination for audio files", metavar="PATH")
    (options, args) = parser.parse_args()
    if options.audio:
        if not os.path.isdir(app.config['CHAT_AUDIO_DIR']):
            try:
                os.makedirs(options.audio)
            except OSError as e:
                parser.error("Cannot create audio directory. Error: {}".format(e))
        else:
            app.config['CHAT_AUDIO_DIR'] = options.audio
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
        app.run(host='0.0.0.0', debug=True, ssl_context=context, port=options.port)
    else:
        rospy.init_node("webui_test", anonymous=True)
        facePub = rospy.Publisher("/camera/face_locations", Faces, queue_size=30)
        app.run(host='0.0.0.0', debug=True, port=options.port)
