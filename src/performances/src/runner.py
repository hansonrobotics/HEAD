#!/usr/bin/env python
from threading import Thread, Lock
import Queue
import rospy
import logging
import performances.srv as srv
import json
from std_msgs.msg import String
from blender_api_msgs.msg import SetGesture, EmotionState, Target
from basic_head_api.msg import MakeFaceExpr
from topic_tools.srv import MuxSelect
import time

logger = logging.getLogger('hr.performances')


class Runner:
    def __init__(self):
        logger.info('Starting performances node')

        self.robot_name = rospy.get_param('/robot_name')
        self.running = False
        self.paused = False
        self.pause_time = 0
        self.start_time = 0
        self.start_timestamp = 0
        self.nodes = []
        self.lock = Lock()
        self.queue = Queue.Queue()
        self.worker = Thread(target=self.worker)
        self.services = {
            'head_pau_mux': rospy.ServiceProxy('/' + self.robot_name + '/head_pau_mux/select', MuxSelect)
        }
        self.topics = {
            'look_at': rospy.Publisher('/blender_api/set_face_target', Target, queue_size=1),
            'gaze_at': rospy.Publisher('/blender_api/set_gaze_target', Target, queue_size=1),
            'emotion': rospy.Publisher('/blender_api/set_emotion_state', EmotionState, queue_size=1),
            'gesture': rospy.Publisher('/blender_api/set_gesture', SetGesture, queue_size=1),
            'expression': rospy.Publisher('/blender_api/make_face_expr', MakeFaceExpr, queue_size=1),
            'interaction': rospy.Publisher('/behavior_switch', String, queue_size=1),
            'tts': {
                'en': rospy.Publisher('/' + self.robot_name + '/chatbot_responses_en', String, queue_size=1),
                'zh': rospy.Publisher('/' + self.robot_name + '/chatbot_responses_zh', String, queue_size=1),
                'default': rospy.Publisher('/' + self.robot_name + '/chatbot_responses', String, queue_size=1),
            }
        }

        rospy.init_node('performances')
        rospy.Service('~run', srv.Run, self.run)
        rospy.Service('~resume', srv.Resume, self.resume)
        rospy.Service('~pause', srv.Pause, self.pause)
        rospy.Service('~stop', srv.Stop, self.stop)

        self.worker.start()
        self.worker.join()

    def resume(self, request):
        success = False
        next_stop_time = 0
        timestamp = 0

        with self.lock:
            if self.running and self.paused:
                current = time.time()
                run_time = self.start_time + self.pause_time - self.start_timestamp

                self.paused = False
                self.start_timestamp = current - run_time

                next_stop_time = self.get_next_stop_time(self.nodes, run_time)
                timestamp = current + next_stop_time - run_time
                success = True

        return srv.ResumeResponse(success, next_stop_time, timestamp)

    def stop(self, request):
        stop_time = 0
        timestamp = 0

        with self.lock:
            if self.running:
                self.running = False
                self.paused = False
                timestamp = time.time()
                stop_time = timestamp - self.start_timestamp

        return srv.StopResponse(True, stop_time, timestamp)

    def pause(self, request):
        with self.lock:
            if self.running and not self.paused:
                self.pause_time = time.time()
                self.paused = True

                return srv.PauseResponse(True, self.pause_time - self.start_timestamp, self.pause_time)
            else:
                return srv.PauseResponse(False, 0, 0)

    @staticmethod
    def get_next_stop_time(nodes, start_time):
        for node in nodes:
            node_stop_time = node['start_time'] + node['duration'] if 'duration' in node else 0
            if node == nodes[-1]:
                return node_stop_time
            elif node['start_time'] < start_time:
                continue
            elif node['name'] == 'pause':
                return node['start_time']

        return 0

    def run(self, request):
        start_time = request.startTime
        nodes = self.sort_nodes(json.loads(request.nodes))
        nodes = [node for node in nodes if node['start_time'] >= start_time]

        with self.lock:
            if not self.running and len(nodes) > 0:
                self.running = True
                self.start_time = start_time
                self.start_timestamp = time.time()
                self.queue.put(nodes)

                next_stop_time = self.get_next_stop_time(nodes, self.start_time)
                next_stop_timestamp = self.start_timestamp + next_stop_time - start_time

                return srv.RunResponse(True, next_stop_time, next_stop_timestamp)
            else:
                return srv.RunResponse(False, 0, 0)

    def worker(self):
        while True:
            with self.lock:
                self.running = False

            self.nodes = self.queue.get()

            with self.lock:
                size = len(self.nodes)

            i = 0
            while i < size:
                with self.lock:
                    if not self.running:
                        break
                    elif self.paused:
                        continue

                    node = self.nodes[i]
                    name = node['name']

                run_time = self.start_time + time.time() - self.start_timestamp
                remaining_time = node['start_time'] - run_time
                if remaining_time > 0:
                    continue

                if name == 'speech':
                    self.speech(node['text'], node['lang'])
                elif name == 'gesture':
                    self.topics['gesture'].publish(
                            SetGesture(node['gesture'], 1, float(node['speed']), float(node['magnitude'])))
                elif name == 'emotion':
                    if 'emotion' in node and 'magnitude' in node:
                        self.topics['emotion'].publish(
                                EmotionState(node['emotion'], float(node['magnitude']),
                                             rospy.Duration.from_sec(node['duration'])))
                elif name == 'look_at':
                    self.topics['look_at'].publish(Target(node['x'], node['y'], node['z']))
                elif name == 'gaze_at':
                    self.topics['gaze_at'].publish(Target(node['x'], node['y'], node['z']))
                elif name == 'interaction':
                    # enable behavior tree
                    self.topics['interaction'].publish(String('btree_on'))

                    # add behavior tree disable node
                    with self.lock:
                        self.nodes.append({'name': 'interaction_end', 'start_time': run_time + node['duration'],
                                           'duration': 0})
                        self.nodes = self.sort_nodes(self.nodes)
                        size += 1
                elif name == 'interaction_end':
                    self.topics['interaction'].publish(String('btree_off'))
                elif name == 'expression':
                    self.services['head_pau_mux']({'topic': "/" + self.robot_name + "/no_pau"})
                    # sleep for 50ms
                    time.sleep(0.05)
                    self.topics['expression'].publish(MakeFaceExpr(node['expression'], float(node['magnitude'])))
                elif name == 'pause':
                    with self.lock:
                        self.paused = True
                        self.pause_time = time.time()
                with self.lock:
                    if node == self.nodes[-1] and name != 'end':
                        end_time = node['start_time'] + node['duration'] if 'duration' in node else 1
                        self.nodes.append({'name': 'end', 'start_time': end_time, 'duration': 0})
                        size += 1

                i += 1

    @staticmethod
    def sort_nodes(nodes):
        return sorted(nodes, key=lambda n: n['start_time'])

    def speech(self, text, lang):
        if lang not in ['en', 'zh']:
            lang = 'default'

        self.topics['tts'][lang].publish(String(text))


if __name__ == '__main__':
    Runner()
