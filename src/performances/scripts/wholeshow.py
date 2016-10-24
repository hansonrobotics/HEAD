#!/usr/bin/env python
import logging

from transitions import *
from transitions.extensions import HierarchicalMachine
import rospy
import rospkg
import os
import yaml
import random
from std_msgs.msg import String
from blender_api_msgs.msg import Target, SomaState
import time
import performances.srv as srv
from performances.msg import Event
import subprocess

logger = logging.getLogger('hr.performance.wholeshow')
rospack = rospkg.RosPack()


class WholeShow(HierarchicalMachine):

    def __init__(self):
        # States for wholeshow
        states = [{'name': 'sleeping', 'children': ['shutting']},
                  {'name': 'interacting', 'children': ['nonverbal']},
                  'performing']
        HierarchicalMachine.__init__(self, states=states, initial='interacting')
        # Transitions
        self.add_transition('wake_up', 'sleeping', 'interacting')
        # Transitions
        self.add_transition('perform', 'interacting', 'performing')
        self.add_transition('shut', 'sleeping', 'sleeping_shutting')
        self.add_transition('be_quiet', 'interacting', 'interacting_nonverbal')
        self.add_transition('start_talking', 'interacting_nonverbal', 'interacting')
        # States handling
        self.on_enter_sleeping("start_sleeping")
        self.on_exit_sleeping("stop_sleeping")
        self.on_enter_interacting("start_interacting")
        self.on_exit_interacting("stop_interacting")
        self.on_enter_sleeping_shutting("system_shutdown")
        # ROS Handling
        rospy.init_node('WholeShow')
        self.btree_pub = rospy.Publisher("/behavior_switch", String, queue_size=2)
        self.soma_pub = rospy.Publisher('/blender_api/set_soma_state', SomaState, queue_size=10)
        self.look_pub = rospy.Publisher('/blender_api/set_face_target', Target, queue_size=10)
        self.performance_runner = rospy.ServiceProxy('/performances/run_full_performance', srv.RunByName)
        # Start sleeping. Wait for Blender to load
        rospy.wait_for_service('/blender_api/get_animation_length')
        rospy.wait_for_service('/performances/current')
        time.sleep(2)
        self.sleeping = rospy.get_param('start_sleeping', False)
        if self.sleeping:
            self.to_sleeping()
        # Performance id as key and keyword array as value
        self.performances_keywords = {}
        # Parse on load.
        # TODO make sure we reload those once performances are saved.
        self.find_performances_properties()
        self.after_performance = False

        # Start listeners
        rospy.Service('speech_on', srv.SpeechOn, self.speech_cb)
        self.sub_sleep = rospy.Subscriber('sleeper', String, self.sleep_cb)
        self.performance_events = rospy.Subscriber('/performances/events', Event, self.performances_cb)
    """States callbacks """
    def start_sleeping(self):
        self.soma_pub.publish(self._get_soma('sleep', 1))
        self.soma_pub.publish(self._get_soma('normal', 0))
        # Look down

        self.look_pub.publish(Target(1, 0, -0.15, 0))

    def stop_sleeping(self):
        self.soma_pub.publish(self._get_soma('sleep', 0))
        self.soma_pub.publish(self._get_soma('normal', 1))
        self.look_pub.publish(Target(1, 0, 0, 0))

    def start_interacting(self):
        self.btree_pub.publish(String("btree_on"))

    def stop_interacting(self):
        self.btree_pub.publish(String("btree_off"))

    """ ROS Callbacks """
    def speech_cb(self, req):
        speech = req.speech
        on = (self.current_state.name == 'interacting')
        # Special states keywords
        if 'go to sleep' in speech:
            try:
                # use to_performng() instead of perform() so it can be called from other than interaction states
                self.to_performing()
                self.after_performance = self.to_sleeping
                self.performance_runner('shared/sleep')
                return srv.SpeechOnResponse(False)
            except:
                pass
        if 'wake' in speech or \
                'makeup' in speech:
            try:
                self.do_wake_up()
                return srv.SpeechOnResponse(False)
            except:
                pass
        if 'shutdown' in speech:
            try:
                self.shut()
                return srv.SpeechOnResponse(False)
            except:
                pass
        if 'be quiet' in speech:
            try:
                self.be_quiet()
                return srv.SpeechOnResponse(False)
            except:
                pass
        if 'hi sophia' in speech or \
                'hey sophia' in speech or \
                'hello sofia' in speech or \
                'hello sophia' in speech or \
                'hi sofia' in speech or \
                'hey sofia' in speech:
            try:
                self.start_talking()
                return srv.SpeechOnResponse(True)
            except:
                pass
            # Try wake up
            try:
                self.do_wake_up()
                return srv.SpeechOnResponse(False)
            except:
                pass

        performances = self.find_performance_by_speech(speech)
        if len(performances) > 0:
            try:
                self.perform()
                on = False
                running = self.performance_runner(random.choice(performances))
            except:
                pass
        return srv.SpeechOnResponse(on)

    def performances_cb(self, msg):
        if msg.event == 'running':
            self.to_performing()
        if msg.event == 'finished'\
                or msg.event == 'idle':
            if self.after_performance:
                self.after_performance()
                self.after_performance = None
            else:
                self.to_interacting()

    def sleep_cb(self, msg):
        if msg.data == 'sleep':
            self.to_sleeping()
        if msg.data == 'wake':
            try:
                self.wake_up()
            except:
                pass

    def do_wake_up(self):
        assert (self.current_state.name == 'sleeping')
        self.after_performance = self.to_interacting
        # Start performance before triggerring state change so soma state will be sinced with performance
        self.performance_runner('shared/wakeup')

    """ Speech"""
    @staticmethod
    def _get_soma(name, magnitude):
        s = SomaState()
        s.name = name
        s.ease_in.secs = 0
        s.ease_in.nsecs = 0.1 * 1000000000
        s.magnitude = magnitude
        s.rate = 1
        return s

    @staticmethod
    def system_shutdown():
        subprocess.call(['sudo', 'shutdown', '-P', 'now'])

    """ Finds properties files for performances and adds keywords for execution """
    def find_performances_properties(self):
        robot_name = rospy.get_param('/robot_name')
        robot_path = os.path.join(rospack.get_path('robots_config'), robot_name, 'performances')
        common_path = os.path.join(rospack.get_path('robots_config'), robot_name, 'performances')
        for path in [common_path, robot_path]:
            for root, dirnames, filenames in os.walk(path):
                if '.properties' in filenames:
                    self.add_performance(path, os.path.relpath(root, path))

    def add_performance(self, path, performance):
        properties_file = os.path.join(path, performance, '.properties')
        with open(properties_file, 'r') as f:
            properties = yaml.load(f.read())
        if 'keywords' in properties.keys() and properties['keywords']:
            self.performances_keywords[performance] = properties['keywords']

    """ Finds performances which one of keyword matches"""
    def find_performance_by_speech(self, speech):
        performances = []
        for performance, keywords in self.performances_keywords.items():
            for keyword in keywords:
                # Currently only simple matching
                if keyword in speech:
                    performances.append(performance)
        return performances


if __name__ == '__main__':
    WholeShow()
    rospy.spin()
