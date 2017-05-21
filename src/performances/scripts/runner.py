#!/usr/bin/env python
from threading import Thread, Lock, Condition
import Queue
import rospy
import logging
import json
import time
import yaml
import os
import fnmatch
import random
import copy
from natsort import natsorted, ns
from std_msgs.msg import String, Int32, Float32
from std_srvs.srv import Trigger, TriggerResponse
from chatbot.msg import ChatMessage
from blender_api_msgs.msg import SetGesture, EmotionState, Target, SomaState
from basic_head_api.msg import MakeFaceExpr, PlayAnimation
from topic_tools.srv import MuxSelect
from performances.nodes import Node
from performances.weak_method import WeakMethod
from performances.msg import Event
import performances.srv as srv
from dynamic_reconfigure.server import Server
from performances.cfg import PerformancesConfig

logger = logging.getLogger('hr.performances')

class Runner:
    def __init__(self):
        logger.info('Starting performances node')

        self.robot_name = rospy.get_param('/robot_name')
        self.robots_config_dir = rospy.get_param('/robots_config_dir')
        self.running = False
        self.paused = False
        self.autopause = False
        self.pause_time = 0
        self.start_time = 0
        self.start_timestamp = 0
        self.lock = Lock()
        self.run_condition = Condition()
        self.running_performance = None
        self.unload_finished = False
        # in memory set of properties with priority over params
        self.variables = {}
        # References to event subscribing node callbacks
        self.observers = {}
        # Performances that already played as alternatives. Used to maximize different performance in single demo
        self.performances_played = {}
        self.worker = Thread(target=self.worker)
        self.worker.setDaemon(True)
        rospy.init_node('performances')
        self.services = {
            'head_pau_mux': rospy.ServiceProxy('/' + self.robot_name + '/head_pau_mux/select', MuxSelect),
            'neck_pau_mux': rospy.ServiceProxy('/' + self.robot_name + '/neck_pau_mux/select', MuxSelect),
            'eyes_pau_mux': rospy.ServiceProxy('/' + self.robot_name + '/eyes_pau_mux/select', MuxSelect)
        }
        self.topics = {
            'running_performance': rospy.Publisher('~running_performance', String, queue_size=1),
            'look_at': rospy.Publisher('/blender_api/set_face_target', Target, queue_size=1),
            'gaze_at': rospy.Publisher('/blender_api/set_gaze_target', Target, queue_size=1),
            'head_rotation': rospy.Publisher('/blender_api/set_head_rotation', Float32, queue_size=1),
            'emotion': rospy.Publisher('/blender_api/set_emotion_state', EmotionState, queue_size=3),
            'gesture': rospy.Publisher('/blender_api/set_gesture', SetGesture, queue_size=3),
            'expression': rospy.Publisher('/' + self.robot_name + '/make_face_expr', MakeFaceExpr, queue_size=3),
            'kfanimation': rospy.Publisher('/' + self.robot_name + '/play_animation', PlayAnimation, queue_size=3),
            'interaction': rospy.Publisher('/behavior_switch', String, queue_size=1),
            'bt_control': rospy.Publisher('/behavior_control', Int32, queue_size=1),
            'events': rospy.Publisher('~events', Event, queue_size=1),
            'chatbot': rospy.Publisher('/' + self.robot_name + '/speech', ChatMessage, queue_size=1),
            'speech_events': rospy.Publisher('/' + self.robot_name + '/speech_events', String, queue_size=1),
            'soma_state': rospy.Publisher("/blender_api/set_soma_state", SomaState, queue_size=2),
            'tts': {
                'en': rospy.Publisher('/' + self.robot_name + '/tts_en', String, queue_size=1),
                'zh': rospy.Publisher('/' + self.robot_name + '/tts_zh', String, queue_size=1),
                'default': rospy.Publisher('/' + self.robot_name + '/tts', String, queue_size=1),
            },
            'tts_control': rospy.Publisher('/' + self.robot_name + '/tts_control', String, queue_size=1)
        }
        self.load_properties()
        rospy.Service('~reload_properties', Trigger, self.reload_properties_callback)
        rospy.Service('~set_properties', srv.SetProperties, self.set_properties_callback)
        rospy.Service('~load', srv.Load, self.load_callback)
        rospy.Service('~load_performance', srv.LoadPerformance, self.load_performance_callback)
        rospy.Service('~unload', Trigger, self.unload_callback)
        rospy.Service('~run', srv.Run, self.run_callback)
        rospy.Service('~run_by_name', srv.RunByName, self.run_by_name_callback)
        rospy.Service('~run_full_performance', srv.RunByName, self.run_full_performance_callback)
        rospy.Service('~resume', srv.Resume, self.resume_callback)
        rospy.Service('~pause', srv.Pause, self.pause_callback)
        rospy.Service('~stop', srv.Stop, self.stop)
        rospy.Service('~current', srv.Current, self.current_callback)
        # Shared subscribers for nodes
        rospy.Subscriber('/' + self.robot_name + '/speech_events', String,
                         lambda msg: self.notify('speech_events', msg))
        rospy.Subscriber('/' + self.robot_name + '/speech', ChatMessage, self.speech_callback)
        # Shared subscribers for nodes
        rospy.Subscriber('/hand_events', String, self.hand_callback)
        Server(PerformancesConfig, self.reconfig)
        rospy.Subscriber('/face_training_event', String, self.training_callback)
        self.worker.start()
        rospy.spin()

    def reconfig(self, config, level):
        with self.lock:
            self.autopause = config.autopause

        return config

    def reload_properties_callback(self, request):
        self.load_properties()
        return TriggerResponse(success=True)

    def unload_callback(self, request):
        self.unload()
        return TriggerResponse(success=True)

    def unload(self):
        self.stop()
        with self.lock:
            if self.running_performance:
                self.running_performance = None
                self.topics['running_performance'].publish(String(json.dumps(None)))

    def set_properties_callback(self, request):
        self.set_variable(request.id, json.loads(request.properties))
        return srv.SetPropertiesResponse(success=True)

    def load_callback(self, request):
        return srv.LoadResponse(success=True, performance=json.dumps(self.load(request.id)))

    def load_performance_callback(self, request):
        self.load_performance(json.loads(request.performance))
        return srv.LoadPerformanceResponse(True)

    def run_by_name_callback(self, request):
        self.stop()
        if not self.load(request.id):
            return srv.RunByNameResponse(False)
        return srv.RunByNameResponse(self.run(0.0))

    def run_full_performance_callback(self, request):
        self.stop()
        performances = self.load_folder(request.id) or self.load(request.id)
        if not performances:
            return srv.RunByNameResponse(False)
        return srv.RunByNameResponse(self.run(0.0, unload_finished=True))

    def load_folder(self, id):
        if id.startswith('shared'):
            robot_name = 'common'
        else:
            robot_name = rospy.get_param('/robot_name')
        dir_path = os.path.join(self.robots_config_dir, robot_name, 'performances', id)
        if os.path.isdir(dir_path):
            root, dirs, files = next(os.walk(dir_path))

            files = fnmatch.filter(files, "*.yaml")
            if not files:
                # If no folder is picked one directory
                # Sub-directories are counted as sub-performances
                if not dirs:
                    return []
                if id in self.performances_played:
                    # All performances played. Pick any but last played
                    if set(self.performances_played[id]) == set(dirs):
                        dirs = self.performances_played[id][:-1]
                        self.performances_played[id] = []
                    else:
                        # Pick from not played performances
                        dirs = list(set(dirs) - set(self.performances_played[id]))
                else:
                    self.performances_played[id] = []
                # Pick random performance
                p = random.choice(dirs)
                self.performances_played[id].append(p)
                return self.load_folder(os.path.join(id, p))
            # make names in folder/file format
            return self.load(id)
        return []

    def load(self, id):
        robot_name = 'common' if id.startswith('shared') else rospy.get_param('/robot_name')
        p = os.path.join(self.robots_config_dir, robot_name, 'performances', id)

        if os.path.isdir(p):
            root, dirs, files = next(os.walk(p))
            files = natsorted(fnmatch.filter(files, "*.yaml"), key=lambda f: f.lower())
            ids = ["{}/{}".format(id, f[:-5]) for f in files]
            timelines = [self.get_timeline(i) for i in ids]
            timelines = [t for t in timelines if t]
            performance = {'id': id, 'name': os.path.basename(id), 'path': os.path.dirname(id), 'timelines': timelines,
                           'nodes': self.get_merged_timeline_nodes(timelines)}
        else:
            performance = self.get_timeline(id)

        if performance:
            self.load_performance(performance)
            return performance
        else:
            return None

    def get_timeline(self, id):
        timeline = None
        robot_name = 'common' if id.startswith('shared') else rospy.get_param('/robot_name')
        p = os.path.join(self.robots_config_dir, robot_name, 'performances', id) + '.yaml'

        if os.path.isfile(p):
            with open(p, 'r') as f:
                timeline = yaml.load(f.read())
                timeline['id'] = id
                timeline['name'] = os.path.basename(id)
                timeline['path'] = os.path.dirname(id)
                self.validate_timeline(timeline)
        return timeline

    def get_timeline_duration(self, timeline):
        duration = 0

        if 'nodes' in timeline and isinstance(timeline['nodes'], list):
            for node in timeline['nodes']:
                duration = max(duration, (node['duration'] if 'duration' in node else 0) + node['start_time'])

        return duration

    def get_merged_timeline_nodes(self, timelines):
        merged = []
        offset = 0

        for timeline in timelines:
            duration = 0
            nodes = timeline.get('nodes', [])
            nodes = copy.deepcopy(nodes)

            for node in nodes:
                duration = max(duration, node['duration'] + node['start_time'])
                node['start_time'] += offset

            merged += nodes
            offset += duration

        return merged

    def validate_performance(self, performance):
        self.validate_timeline(performance)
        if 'timelines' in performance:
            for timeline in performance['timelines']:
                self.validate_timeline(timeline)
        return performance

    def validate_timeline(self, timeline):
        if 'nodes' not in timeline or not isinstance(timeline['nodes'], list):
            timeline['nodes'] = []

        for node in timeline['nodes']:
            if 'start_time' not in node:
                node['start_time'] = 0
            if node['name'] == 'pause':
                node['duration'] = 0.1
            if 'duration' not in node or not node['duration']:
                node['duration'] = 0

        return timeline

    def load_performance(self, performance):
        with self.lock:
            self.validate_performance(performance)
            self.running_performance = performance
            self.topics['running_performance'].publish(String(json.dumps(performance)))

    def run_callback(self, request):
        return srv.RunResponse(self.run(request.startTime))

    def run(self, start_time, unload_finished=False):
        self.stop()
        # Wait for worker to stop performance and enter waiting before proceeding
        self.run_condition.acquire()
        with self.lock:
            success = self.running_performance and len(self.running_performance) > 0
            if success:
                self.unload_finished = unload_finished
                self.running = True
                self.start_time = start_time
                self.start_timestamp = time.time()
                # notify worker thread
                self.run_condition.notify()
            self.run_condition.release()
            return success

    def resume_callback(self, request):
        success = self.resume()
        with self.lock:
            run_time = self.get_run_time()

        return srv.ResumeResponse(success, run_time)

    def resume(self):
        success = False
        with self.lock:
            if self.running and self.paused:
                run_time = self.get_run_time()
                self.paused = False
                self.start_timestamp = time.time() - run_time
                self.start_time = 0
                self.topics['events'].publish(Event('resume', run_time))
                success = True

        return success

    def stop(self, request=None):
        stop_time = 0

        with self.lock:
            if self.running:
                stop_time = self.get_run_time()
                self.running = False
                self.paused = False
                self.topics['tts_control'].publish('shutup')

        return srv.StopResponse(True, stop_time)

    def pause_callback(self, request):
        if self.pause():
            with self.lock:
                return srv.PauseResponse(True, self.get_run_time())
        else:
            return srv.PauseResponse(False, 0)

    # Pauses current
    def pause(self):
        with self.lock:
            if self.running and not self.paused:
                self.pause_time = time.time()
                self.paused = True
                self.topics['events'].publish(Event('paused', self.get_run_time()))
                return True
            else:
                return False

    # Returns current performance
    def current_callback(self, request):
        with self.lock:
            current_time = self.get_run_time()
            running = self.running and not self.paused
            return srv.CurrentResponse(performance=json.dumps(self.running_performance),
                                       current_time=current_time,
                                       running=running)

    def worker(self):
        self.run_condition.acquire()
        while True:
            with self.lock:
                self.paused = False
                self.running = False

            self.topics['events'].publish(Event('idle', 0))
            self.run_condition.wait()
            self.topics['events'].publish(Event('running', self.start_time))

            with self.lock:
                if not self.running_performance:
                    continue

            behavior = True
            offset = 0
            timelines = self.running_performance['timelines'] if 'timelines' in self.running_performance else [
                self.running_performance]

            for i, timeline in enumerate(timelines):
                # check if performance is finished without starting
                running = True
                nodes = [Node.createNode(node, self, self.start_time - offset, timeline.get('id', '')) for node in
                         timeline['nodes']]
                pid = timeline.get('id', '')
                finished = None
                pause = pid and self.get_property(os.path.dirname(pid), 'pause_behavior')
                # Pause must be either enabled or not set (by default all performances are
                # pausing behavior if its not set)

                if (pause or pause is None) and behavior:
                    # Only pause behavior if its already running. Otherwise Pause behavior have no effect
                    behavior_enabled = False

                    try:
                        behavior_enabled = rospy.get_param("/behavior_enabled")
                    except KeyError:
                        pass

                    if behavior_enabled:
                        self.topics['interaction'].publish('btree_off')
                        behavior = False

                with self.lock:
                    if not self.running:
                        break

                while running:
                    with self.lock:
                        run_time = self.get_run_time()

                        if not self.running:
                            self.topics['events'].publish(Event('finished', run_time))
                            break

                        if self.paused:
                            continue

                    running = False
                    # checks if any nodes still running
                    for k, node in enumerate(nodes):
                        running = node.run(run_time - offset) or running
                    if finished is None:
                        # true if all performance nodes are already finished
                        finished = not running

                offset += self.get_timeline_duration(timeline)

                with self.lock:
                    autopause = self.autopause and finished is False and i < len(timelines) - 1

                if autopause:
                    self.pause()

            if not behavior:
                self.topics['interaction'].publish('btree_on')

            if self.unload_finished:
                self.unload_finished = False
                self.unload()

    def get_run_time(self):
        """
        Must acquire self.lock in order to safely use this method
        :return:
        """
        run_time = 0

        if self.running:
            run_time = self.start_time
            if self.paused:
                run_time += self.pause_time - self.start_timestamp
            else:
                run_time += time.time() - self.start_timestamp

        return run_time

    # Notifies register nodes on the events from ROS.
    def notify(self, event, msg):
        if event not in self.observers.keys():
            return
        for i in xrange(len(self.observers[event]) - 1, -1, -1):
            try:
                self.observers[event][i](msg)
            except TypeError:
                # Remove dead methods
                del self.observers[event][i]

    # Registers callbacks for specific events. Uses weak reference to allow nodes cleanup after finish.
    def register(self, event, cb):
        if not event in self.observers:
            self.observers[event] = []
        m = WeakMethod(cb)
        self.observers[event].append(m)
        return m

    # Allows nodes to unsubscribe from events
    def unregister(self, event, ref):
        if event in self.observers:
            if ref in self.observers[event]:
                self.observers[event].remove(ref)

    def hand_callback(self, msg):
        self.notify('HAND', msg)
        self.notify(msg.data, msg)

    def load_properties(self):
        robot_name = rospy.get_param('/robot_name')
        robot_path = os.path.join(self.robots_config_dir, robot_name, 'performances')
        common_path = os.path.join(self.robots_config_dir, 'common', 'performances')
        for path in [common_path, robot_path]:
            for root, dirnames, filenames in os.walk(path):
                if '.properties' in filenames:
                    filename = os.path.join(root, '.properties')
                    if os.path.isfile(filename):
                        with open(filename) as f:
                            properties = yaml.load(f.read())
                            dir = os.path.relpath(root, path)
                            rospy.set_param('/' + os.path.join(self.robot_name, 'webui/performances', dir).strip(
                                "/.") + '/properties', properties)

    def get_property(self, path, name):
        param_name = os.path.join('/', self.robot_name, 'webui/performances', path, 'properties', name)
        return rospy.get_param(param_name, None)

    def set_variable(self, id, properties):
        for key, val in properties.iteritems():
            rospy.logerr("id {} key {} val {}".format(id, key, val))
            if id in self.variables:
                self.variables[id][key] = val
            else:
                self.variables[id] = {key: val}

    def get_variable(self, id, name):
        if os.path.dirname(id) in self.variables and name in self.variables[os.path.dirname(id)] \
                and self.variables[os.path.dirname(id)][name]:
            return self.variables[os.path.dirname(id)][name]
        else:
            val = None
            param_name = os.path.join('/', self.robot_name, 'webui/performances', os.path.dirname(id),
                                      'properties/variables', name)
            if rospy.has_param(param_name):
                val = rospy.get_param(param_name)
                if self.is_param(val):
                    if rospy.has_param(val):
                        return str(rospy.get_param(val))
                    if rospy.has_param("/{}{}".format(self.robot_name, val)):
                        return str(rospy.get_param("/{}{}".format(self.robot_name, val)))

                    return None
            return val

    def speech_callback(self, msg):
        self.notify('SPEECH', msg.utterance)

    @staticmethod
    def is_param(param):
        """ Checks if value is valid param.
        Has to start with slash
        """
        validator = rospy.names.global_name("param_name")
        try:
            validator(param, False)
            return True
        except rospy.names.ParameterInvalid:
            return False

    def training_callback(self, msg):
        self.notify('FACE_TRAINING', msg.data)


if __name__ == '__main__':
    Runner()
