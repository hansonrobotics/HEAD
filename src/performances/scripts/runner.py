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

from std_msgs.msg import String, Int32, Float32
from chatbot.msg import ChatMessage
from blender_api_msgs.msg import SetGesture, EmotionState, Target, SomaState
from basic_head_api.msg import MakeFaceExpr, PlayAnimation
from topic_tools.srv import MuxSelect
from performances.nodes import Node
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
        self.queue = Queue.Queue()
        self.ids = []
        self.running_nodes = []
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
        rospy.Service('~load', srv.Load, self.load_callback)
        rospy.Service('~load_nodes', srv.LoadNodes, self.load_nodes_callback)
        rospy.Service('~load_sequence', srv.LoadSequence, self.load_sequence_callback)
        rospy.Service('~run', srv.Run, self.run_callback)
        rospy.Service('~run_by_name', srv.RunByName, self.run_by_name_callback)
        rospy.Service('~resume', srv.Resume, self.resume_callback)
        rospy.Service('~pause', srv.Pause, self.pause_callback)
        rospy.Service('~stop', srv.Stop, self.stop)
        rospy.Service('~current', srv.Current, self.current_callback)
        self.worker.start()
        rospy.spin()

    def load_callback(self, request):
        return srv.LoadResponse(success=True, nodes=json.dumps(self.load_sequence([request.id])))

    def load_nodes_callback(self, request):
        self.load_nodes(json.loads(request.nodes))
        return srv.LoadNodesResponse(True)

    def load_sequence_callback(self, request):
        return srv.LoadSequenceResponse(success=True, nodes=json.dumps(self.load_sequence(request.ids)))

    def run_by_name_callback(self, request):
        self.stop()
        nodes = self.load_sequence([request.id])
        if not nodes:
            return srv.RunByNameResponse(False)
        return srv.RunByNameResponse(self.run(0.0))


    def load_sequence(self, ids):
        nodes = []

        offset = 0
        for id in ids:
            robot_name = rospy.get_param('/robot_name')
            path = os.path.join(rospack.get_path('robots_config'), robot_name, 'performances', id + '.yaml')
            duration = 0

            if os.path.isfile(path):
                with open(path, 'r') as f:
                    data = yaml.load(f.read())

                if 'nodes' in data and isinstance(data['nodes'], list):
                    for node in data['nodes']:
                        if not 'start_time' in node:
                            node['start_time'] = 0
                        duration = max(duration, (node['duration'] if 'duration' in node else 0) + node['start_time'])
                        node['start_time'] += offset
                    offset += duration
                    nodes += data['nodes']

        self.load_nodes(nodes, ids)
        return nodes

    def load_nodes(self, nodes, ids=None):
        self.stop()

        if ids is None:
            ids = []

        with self.lock:
            self.ids = ids
            self.running_nodes = nodes

    def load(self, nodes, ids=None):
        if ids is None:
            ids = []

        with self.lock:
            logger.info("Put nodes {} to queue".format(nodes))
            self.running_nodes = nodes
            self.ids = ids

    def run_callback(self, request):
        return srv.RunResponse(self.run(request.startTime))

    def run(self, start_time):
        self.stop()

        with self.lock:
            if len(self.running_nodes) > 0:
                self.running = True
                self.start_time = start_time
                self.start_timestamp = time.time()
                # notify worker thread
                self.run_condition.acquire()
                self.run_condition.notify()
                self.run_condition.release()
                return True
            else:
                return False

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
            return srv.CurrentResponse(ids=self.ids, nodes=json.dumps(self.running_nodes), current_time=current_time,
                                       running=running)

    def worker(self):
        self.run_condition.acquire()
        while True:
            with self.lock:
                self.paused = False

            self.topics['events'].publish(Event('idle', 0))
            self.run_condition.wait()
            with self.lock:
                nodes = [Node.createNode(node, self, self.start_time) for node in self.running_nodes]
            self.topics['events'].publish(Event('running', self.start_time))

            if len(nodes) == 0:
                continue
            running = True
            while running:
                with self.lock:
                    run_time = self.get_run_time()

                    if not self.running:
                        self.topics['events'].publish(Event('finished', run_time))
                        break
                    elif self.paused:
                        continue

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


if __name__ == '__main__':
    Runner()
