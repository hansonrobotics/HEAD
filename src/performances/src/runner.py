#!/usr/bin/env python
from threading import Thread, Lock
import Queue
import rospy
import logging
import performances.srv as srv
import json
from std_msgs.msg import String
from blender_api_msgs.msg import SetGesture, EmotionState, Target
from basic_head_api.msg import MakeFaceExpr, PlayAnimation
from topic_tools.srv import MuxSelect
from performances.msg import Event
import time
from nodes import Node
import rospkg
import yaml
import os

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
        self.queue = Queue.Queue()
        self.worker = Thread(target=self.worker)
        self.worker.setDaemon(True)
        rospy.init_node('performances')
        self.services = {
            'head_pau_mux': rospy.ServiceProxy('/' + self.robot_name + '/head_pau_mux/select', MuxSelect)
        }
        self.topics = {
            'look_at': rospy.Publisher('/blender_api/set_face_target', Target, queue_size=1),
            'gaze_at': rospy.Publisher('/blender_api/set_gaze_target', Target, queue_size=1),
            'emotion': rospy.Publisher('/blender_api/set_emotion_state', EmotionState, queue_size=3),
            'gesture': rospy.Publisher('/blender_api/set_gesture', SetGesture, queue_size=3),
            'expression': rospy.Publisher('/' + self.robot_name + '/make_face_expr', MakeFaceExpr, queue_size=3),
            'kfanimation': rospy.Publisher('/' + self.robot_name + '/play_animation', PlayAnimation, queue_size=3),
            'interaction': rospy.Publisher('/behavior_switch', String, queue_size=1),
            'events': rospy.Publisher('~events', Event, queue_size=1),
            'tts': {
                'en': rospy.Publisher('/' + self.robot_name + '/chatbot_responses_en', String, queue_size=1),
                'zh': rospy.Publisher('/' + self.robot_name + '/chatbot_responses_zh', String, queue_size=1),
                'default': rospy.Publisher('/' + self.robot_name + '/chatbot_responses', String, queue_size=1),
            }
        }
        rospy.Service('~run_by_name', srv.RunByName, self.run_by_name)
        rospy.Service('~run', srv.Run, self.run)
        rospy.Service('~resume', srv.Resume, self.resume_callback)
        rospy.Service('~pause', srv.Pause, self.pause_callback)
        rospy.Service('~stop', srv.Stop, self.stop)
        self.worker.start()
        rospy.spin()

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

    def run_by_name(self, request):
        name = request.name
        robot_name = rospy.get_param('/robot_name')
        path = os.path.join(rospack.get_path('performances'), 'robots', robot_name, name + '.yaml')

        if os.path.exists(path):
            with open(path, 'r') as f:
                data = yaml.load(f.read())

            if 'nodes' in data and isinstance(data['nodes'], list):
                return srv.RunByNameResponse(self.run(0, data['nodes']))

        return srv.RunByNameResponse(False)

    def run_callback(self, request):
        start_time = request.startTime
        nodes = json.loads(request.nodes)

        return srv.RunResponse(self.run(start_time, nodes))

    def run(self, start_time, nodes):
        # Create nodes
        nodes = [Node.createNode(node, self, start_time) for node in nodes]
        # Stop running first if performance is running
        size = len(nodes)
        if size > 0:
            self.stop()

        with self.lock:
            if size > 0:
                self.running = True
                self.start_time = start_time
                self.start_timestamp = time.time()
                self.queue.put(nodes)
                return True
            else:
                return False

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

    def worker(self):
        while True:
            with self.lock:
                self.paused = False

            self.topics['events'].publish(Event('idle', 0))
            nodes = self.queue.get()
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
                for node in nodes:
                    running = node.run(run_time) or running

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
