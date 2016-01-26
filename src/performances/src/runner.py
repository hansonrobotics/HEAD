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
from nodes import Node

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
        self.worker.setDaemon(True)
        rospy.init_node('performances')
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
            'performance_status': rospy.Publisher('/behavior_switch', String, queue_size=1),
            'tts': {
                'en': rospy.Publisher('/' + self.robot_name + '/chatbot_responses_en', String, queue_size=1),
                'zh': rospy.Publisher('/' + self.robot_name + '/chatbot_responses_zh', String, queue_size=1),
                'default': rospy.Publisher('/' + self.robot_name + '/chatbot_responses', String, queue_size=1),
            }
        }
        rospy.Service('~run', srv.Run, self.run)
        rospy.Service('~resume', srv.Resume, self.resume)
        rospy.Service('~pause', srv.Pause, self.pause)
        rospy.Service('~stop', srv.Stop, self.stop)
        self.worker.start()
        rospy.spin()

    def resume(self, request):
        success = False
        with self.lock:
            if self.running and self.paused:
                current = time.time()
                run_time = self.start_time + self.pause_time - self.start_timestamp
                self.paused = False
                self.start_timestamp = current - run_time
                success = True

        return srv.ResumeResponse(success, 0, 0)

    def stop(self, request=None):
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

    def run(self, request):
        start_time = request.startTime
        nodes = json.loads(request.nodes)
        # Create nodes
        nodes = [Node.createNode(node, self, start_time) for node in nodes]
        # Stop running first if performance is running
        if self.running and len(nodes) > 0:
            self.stop()
            
        with self.lock:
            if len(nodes) > 0:
                self.running = True
                self.start_time = start_time
                self.start_timestamp = time.time()
                self.queue.put(nodes)
                return srv.RunResponse(True, 0, 0)
            else:
                return srv.RunResponse(False, 0, 0)

    # Pauses current
    def make_pause(self):
        with self.lock:
            if self.running and not self.paused:
                self.pause_time = time.time()
                self.paused = True
                return True
            else:
                return False

    def worker(self):
        while True:
            with self.lock:
                self.running = False
            self.nodes = self.queue.get()
            if len(self.nodes) == 0:
                continue
            running = True
            while running:
                if not self.running:
                    break
                elif self.paused:
                    continue
                run_time = self.start_time + time.time() - self.start_timestamp
                running = False
                for node in self.nodes:
                    running = node.run(run_time) or running


if __name__ == '__main__':
    Runner()
