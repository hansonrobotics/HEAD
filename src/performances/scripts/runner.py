#!/usr/bin/env python
from threading import Thread, Lock, Condition
import Queue
import rospy
import logging
import json
import time
import rospkg
import yaml
import os
import fnmatch
import random

from std_msgs.msg import String, Int32, Float32
from std_srvs.srv import Trigger
from chatbot.msg import ChatMessage
from blender_api_msgs.msg import SetGesture, EmotionState, Target, SomaState
from basic_head_api.msg import MakeFaceExpr, PlayAnimation
from topic_tools.srv import MuxSelect
from performances.nodes import Node
from performances.weak_method import WeakMethod
from performances.msg import Event
import performances.srv as srv

logger = logging.getLogger('hr.performances')
rospack = rospkg.RosPack()


class Runner:
    def __init__(self):
        logger.info('Starting performances node')

        self.robot_name = rospy.get_param('/robot_name')
        self.running = False
        self.paused = False
        self.pause_time = 0
        self.start_time = 0
        self.start_timestamp = 0
        self.lock = Lock()
        self.run_condition = Condition()
        self.running_performances = []
        # in memory set of properties with priority over params
        self.properties = {}
        # References to event subscribing node callbacks
        self.observers = {}
        # Performances that already played as alternatives. Used to maximize different performance in single demo
        self.performances_played = {}
        self.worker = Thread(target=self.worker)
        self.worker.setDaemon(True)
        rospy.init_node('performances')
        self.services = {
            'head_pau_mux': rospy.ServiceProxy('/' + self.robot_name + '/head_pau_mux/select', MuxSelect),
            'neck_pau_mux': rospy.ServiceProxy('/' + self.robot_name + '/neck_pau_mux/select', MuxSelect)
        }
        self.topics = {
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
            }
        }
        self.load_properties()
        rospy.Service('~reload_properties', Trigger, self.reload_properties_callback)
        rospy.Service('~set_property', srv.SetProperty, self.set_property_callback)
        rospy.Service('~load', srv.Load, self.load_callback)
        rospy.Service('~load_sequence', srv.LoadSequence, self.load_sequence_callback)
        rospy.Service('~load_performance', srv.LoadPerformance, self.load_performance_callback)
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
        # Shared subscribers for nodes
        rospy.Subscriber('/hand_events', String, self.hand_callback)

        self.worker.start()
        rospy.spin()

    def reload_properties_callback(self, request):
        self.load_properties()
        return Trigger(success=True)

    def set_property_callback(self, request):
        self.set_property(request.id, request.name, request.val)
        return srv.SetPropertyResponse(success=True)

    def load_callback(self, request):
        return srv.LoadResponse(success=True, performance=json.dumps(self.load_sequence([request.id])[0]))

    def load_performance_callback(self, request):
        self.load_performances(json.loads(request.performance))
        return srv.LoadPerformanceResponse(True)

    def load_sequence_callback(self, request):
        return srv.LoadSequenceResponse(success=True, performances=json.dumps(self.load_sequence(request.ids)))

    def run_by_name_callback(self, request):
        self.stop()
        performances = self.load_sequence([request.id])
        if not performances:
            return srv.RunByNameResponse(False)
        return srv.RunByNameResponse(self.run(0.0))

    def run_full_performance_callback(self, request):
        self.stop()
        performances = self.load_folder(request.id)
        if not performances:
            return srv.RunByNameResponse(False)
        return srv.RunByNameResponse(self.run(0.0))

    def load_folder(self, performance):
        if performance.startswith('shared'):
            robot_name = 'common'
        else:
            robot_name = rospy.get_param('/robot_name')
        dir_path = os.path.join(rospack.get_path('robots_config'), robot_name, 'performances', performance)
        if os.path.isdir(dir_path):
            root, dirs, files = next(os.walk(dir_path))
            files = fnmatch.filter(sorted(files), "*.yaml")
            if not files:
                # If no folder is picked one directory
                # Sub-directories are counted as sub-performances
                if not dirs:
                    return []
                if performance in self.performances_played:
                    # All performances played. Pick any but last played
                    if set(self.performances_played[performance]) == set(dirs):
                        dirs = self.performances_played[performance][:-1]
                        self.performances_played[performance] = []
                    else:
                        # Pick from not played performances
                        dirs = list(set(dirs) - set(self.performances_played[performance]))
                else:
                    self.performances_played[performance] = []
                # Pick random performance
                p = random.choice(dirs)
                self.performances_played[performance].append(p)
                return self.load_folder(os.path.join(performance, p))
            # make names in folder/file format
            ids = ["{}/{}".format(performance, f[:-5]) for f in files]
            return self.load_sequence(ids)
        return []

    def load_sequence(self, ids):
        performances = []
        for id in ids:
            if id.startswith('shared'):
                robot_name = 'common'
            else:
                robot_name = rospy.get_param('/robot_name')
            filename = os.path.join(rospack.get_path('robots_config'), robot_name, 'performances', id + ".yaml")

            if os.path.isfile(filename):
                with open(filename, 'r') as f:
                    performance = yaml.load(f.read())
                    performance['id'] = id
                    performance['path'] = os.path.dirname(id)
                    performances.append(performance)

        return self.load_performances(performances)

    def load_performances(self, performances):
        offset = 0

        if not isinstance(performances, list):
            performances = [performances]

        for performance in performances:
            duration = 0

            if 'nodes' in performance and isinstance(performance['nodes'], list):
                for node in performance['nodes']:
                    if not 'start_time' in node:
                        node['start_time'] = 0
                    if node['name'] == 'pause':
                        node['duration'] = 0.1
                    duration = max(duration, (node['duration'] if 'duration' in node else 0) + node['start_time'])
                    node['start_time'] += offset
                offset += duration
            else:
                performances.remove(performance)
        with self.lock:
            self.running_performances = performances

        return performances

    def run_callback(self, request):
        return srv.RunResponse(self.run(request.startTime))

    def run(self, start_time):
        self.stop()
        # Wait for worker to stop performance and enter waiting before proceeding
        self.run_condition.acquire()
        with self.lock:
            success = len(self.running_performances) > 0
            if success:
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
            return srv.CurrentResponse(performances=json.dumps(self.running_performances),
                                       current_time=current_time,
                                       running=running)

    def worker(self):
        self.run_condition.acquire()
        while True:
            with self.lock:
                self.paused = False

            self.topics['events'].publish(Event('idle', 0))
            self.run_condition.wait()
            self.topics['events'].publish(Event('running', self.start_time))

            if len(self.running_performances) == 0:
                continue

            for performance in self.running_performances:
                nodes = [Node.createNode(node, self, self.start_time, performance['id']) for node in
                         performance['nodes']]

                with self.lock:
                    if not self.running:
                        break

                running = True
                while running:
                    with self.lock:
                        run_time = self.get_run_time()

                        if not self.running:
                            self.topics['events'].publish(Event('finished', run_time))
                            break
                    running = False
                    # checks if any nodes still running
                    for node in nodes:
                        running = node.run(run_time) or running
        self.run_condition.release()

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
        robot_path = os.path.join(rospack.get_path('robots_config'), robot_name, 'performances')
        common_path = os.path.join(rospack.get_path('robots_config'), 'common', 'performances')
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

    def set_property(self, id, name, val):
        if id in self.properties:
            self.properties[id][name] = val
        else:
            self.properties[id] = {name: val}

    def get_property(self, id, name):
        if id in self.properties and name in self.properties[id] and self.properties[id][name]:
            return self.properties[id][name]
        else:
            val = None
            param_name = os.path.join('/', self.robot_name, 'webui/performances', os.path.dirname(id),
                                      'properties/variables', name)
            if rospy.has_param(param_name):
                val = rospy.get_param(param_name)
            return val


if __name__ == '__main__':
    Runner()
