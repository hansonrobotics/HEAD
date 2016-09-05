#!/usr/bin/env python

import logging
from transitions import *
import rospy
import yaml
import re
import string
from chatbot.msg import ChatMessage
from std_msgs.msg import String
from blender_api_msgs.msg import EmotionState, SetGesture, Target, SomaState
from threading import Timer
import time
logger = logging.getLogger('hr.performance.wholeshow')
import performances.srv as srv
from performances.msg import Event

class WholeShow(Machine):

    def __init__(self):
        # States for wholeshow
        states = ['sleeping', 'interacting', 'performing']
        Machine.__init__(self, states=states, initial='interacting')
        # Transitions
        self.add_transition('wake_up', 'sleeping', 'interacting')
        # Transitions
        self.add_transition('perform', 'interacting', 'performing')
        # States handling
        self.on_enter_sleeping("start_sleeping")
        self.on_exit_sleeping("stop_sleeping")
        self.on_enter_interacting("start_interacting")
        self.on_exit_interacting("stop_interacting")
        # ROS Handling
        rospy.init_node('WholeShow')
        self.sub_sleep = rospy.Subscriber('sleeper', String, self.sleep_cb)
        self.perfmance_events = rospy.Subscriber('/performances/events', Event, self.performances_cb)
        self.btree_pub = rospy.Publisher("/behavior_switch", String, queue_size=2)
        self.soma_pub = rospy.Publisher('/blender_api/set_soma_state', SomaState, queue_size=10)
        self.look_pub = rospy.Publisher('/blender_api/set_face_target', Target, queue_size=10)
        self.performance_runner = rospy.ServiceProxy('/performances/run_full_perfromance', srv.RunByName)
        # Chat messages
        rospy.Service('speech_on', srv.SpeechOn, self.speech_cb)
        # Start sleeping. Wait for Blender to load
        rospy.wait_for_service('/blender_api/get_animation_length')
        time.sleep(0.1)
        self.to_sleeping()


    """States callbacks """
    def start_sleeping(self):
        self.soma_pub.publish(self._getSoma('sleep', 1))
        self.soma_pub.publish(self._getSoma('normal', 0))
        # Look down

        self.look_pub.publish(Target(1,0,-0.15,0))

    def stop_sleeping(self):
        self.soma_pub.publish(self._getSoma('sleep', 0))
        self.soma_pub.publish(self._getSoma('normal', 1))
        self.look_pub.publish(Target(1, 0, 0, 0))

    def start_interacting(self):
        self.btree_pub.publish(String("btree_on"))

    def stop_interacting(self):
        self.btree_pub.publish(String("btree_off"))

    """ Transition callbacks """
    """ ROS Callbacks """
    def speech_cb(self, req):
        speech = req.speech
        on = (self.current_state.name == 'interacting')
        if 'go to sleep' in speech:
            try:
                print "try sleeping"
                self.to_sleeping()
                print "currently sleeping"
                on = False
            except:
                pass
        if 'wake' in speech or \
            'makeup' in speech:
            try:
                self.wake_up()
                on = False
            except:
                pass
        if 'play rock paper' in speech or \
            'play rock paper' in speech:
            try:
                self.perform()
                self.performance_runner('rps')
                on = False
            except:
                pass
        return srv.SpeechOnResponse(on)

    def performances_cb(self,msg):
        if msg.event == 'running':
            self.to_performing()
        if msg.event == 'finished':
            self.to_interacting()

    def sleep_cb(self, msg):
        if msg.data == 'sleep':
            self.to_sleeping()
        if msg.data == 'wake':
            try:
                self.wake_up()
            except:
                pass

    """ Speech"""
    def _getSoma(self, name, magnitude):
        s = SomaState()
        s.name = name
        s.ease_in.secs = 0
        s.ease_in.nsecs = 0.1 * 1000000000
        s.magnitude = magnitude
        s.rate = 1
        return s

if __name__ == '__main__':
    WholeShow()
    rospy.spin()
