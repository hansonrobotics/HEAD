#!/usr/bin/env python
import logging

from transitions import *
from transitions.extensions import HierarchicalMachine
import rospy
import rospkg
import os
import re
import random
from std_msgs.msg import String
from blender_api_msgs.msg import Target, SomaState
from blender_api_msgs.srv import SetParam
import time
import performances.srv as srv
from performances.msg import Event
import subprocess
import threading
import dynamic_reconfigure.client
from chatbot.msg import ChatMessage
from dynamic_reconfigure.server import Server
from performances.cfg import WholeshowConfig
logger = logging.getLogger('hr.performance.wholeshow')
rospack = rospkg.RosPack()


class WholeShow(HierarchicalMachine):
    OPENCOG_ENTER = ['enable advanced', 'start advanced', 'activate advanced']
    OPENCOG_EXIT = ['disable advanced', 'deactivate advanced', 'exit advanced']

    def __init__(self):
        rospy.wait_for_service('/performances/reload_properties')

        # States for wholeshow
        states = [{'name': 'sleeping', 'children': ['shutting']},
                  {'name': 'interacting', 'children': ['nonverbal']},
                  'performing', 'opencog', 'analysis']
        HierarchicalMachine.__init__(self, states=states, initial='interacting')
        # Transitions
        self.add_transition('wake_up', 'sleeping', 'interacting')
        # Transitions
        self.add_transition('perform', 'interacting', 'performing')
        self.add_transition('interact', 'performing', 'interacting')
        self.add_transition('start_opencog', 'interacting', 'opencog')
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
        self.btree_pub = rospy.Publisher("/behavior_switch", String, queue_size=5)
        self.btree_sub = rospy.Subscriber("/behavior_switch", String, self.btree_cb)
        self.soma_pub = rospy.Publisher('/blender_api/set_soma_state', SomaState, queue_size=10)
        self.look_pub = rospy.Publisher('/blender_api/set_face_target', Target, queue_size=10)
        self.gaze_pub = rospy.Publisher('/blender_api/set_gaze_target', Target, queue_size=10)
        self.performance_runner = rospy.ServiceProxy('/performances/run_full_performance', srv.RunByName)
        # Wholeshow starts with behavior enabled, unless set otherwise
        rospy.set_param("/behavior_enabled", rospy.get_param("/behavior_enabled", True))
        # Start sleeping. Wait for Blender to load
        rospy.wait_for_service('/blender_api/set_param')
        rospy.wait_for_service('/performances/current')
        self.blender_param = rospy.ServiceProxy('/blender_api/set_param', SetParam)
        time.sleep(2)
        self.sleeping = rospy.get_param('start_sleeping', False)
        if self.sleeping:
            t = threading.Timer(1, self.to_sleeping)
            t.start()
        # Performance id as key and keyword array as value
        self.performances_keywords = {}
        # Parse on load.
        # TODO make sure we reload those once performances are saved.
        self.after_performance = False
        # Speech handler. Receives all speech input, and forwards to chatbot if its not a command input,
        #  or chat is enabled
        self.speech_sub = rospy.Subscriber('speech', ChatMessage, self.speech_cb)
        self.speech_pub = rospy.Publisher('chatbot_speech', ChatMessage, queue_size=10)
        # Sleep
        self.performance_events = rospy.Subscriber('/performances/events', Event, self.performances_cb)
        # Dynamic reconfigure
        self.config = {}
        self.cfg_srv = Server(WholeshowConfig, self.config_cb)
        # Behavior was paused entering into state
        self.behavior_paused = False
        # Chatbot was paused entering the state
        self.chatbot_paused = False


    def start_sleeping(self):
        """States callbacks """
        self.btree_pub.publish(String("btree_off"))
        self.soma_pub.publish(self._get_soma('sleep', 1))
        self.soma_pub.publish(self._get_soma('normal', 0))
        # Look down
        self.look_pub.publish(Target(1, 0, -0.15, 0.3))
        self.enable_blinking(False)
        # Update param in case wholeshow restarts
        rospy.set_param('start_sleeping', True)

    def stop_sleeping(self):
        self.soma_pub.publish(self._get_soma('sleep', 0))
        self.soma_pub.publish(self._get_soma('normal', 1))
        self.look_pub.publish(Target(1, 0, 0, 0))
        self.enable_blinking()
        # Update param in case wholeshow restarts
        rospy.set_param('start_sleeping', False)

    def start_interacting(self):
        pass

    def stop_interacting(self):
        pass

    def speech_cb(self, msg):
        """ ROS Callbacks """
        speech = msg.utterance
        on = (self.state == 'interacting') or (self.state == 'performing' and self.config['chat_during_performance'])
        # Special states keywords
        if self.state == 'opencog':
            if self.check_keywords(self.OPENCOG_EXIT, speech):
                self.to_interacting()
            self.speech_pub.publish(msg)
        if self.check_keywords(self.OPENCOG_ENTER, speech):
            try:
                self.start_opencog()
            except Exception:
                pass
            self.speech_pub.publish(msg)
        if 'go to sleep' in speech:
            try:
                self.btree_pub.publish(String("btree_off"))
                # use to_performng() instead of perform() so it can be called from other than interaction states
                self.to_performing()
                self.after_performance = self.to_sleeping
                self.performance_runner('shared/sleep')
                return False
            except:
                pass
        if 'wake' in speech or 'makeup' in speech:
            try:
                self.do_wake_up()
                return False
            except:
                pass
        if 'shutdown' in speech:
            try:
                self.shut()
                return False
            except:
                pass
        if 'be quiet' in speech:
            try:
                self.be_quiet()
                return False
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
                self.speech_pub.publish(msg)
            except:
                pass
            # Try wake up
            try:
                self.do_wake_up()
                return False
            except:
                pass
        if ('go to analysis mode' in speech) or ('analysis mode' in speech) or (speech == 'analysis'):
            try:
                self.to_analysis()
                return True
            except:
                pass

        if ('exit' in speech) or ('return' in speech) or ('normal mode' in speech):
            try:
                rospy.logerr("Exiting interaction mode")
                self.to_interacting()
                return True
            except:
                pass

        performances = self.find_performance_by_speech(speech)

        # Split between performances for general modes and analysis
        analysis_performances = [p for p in performances if ('shared/analysis' in p or 'robot/analysis' in p)]
        for a in analysis_performances:
            performances.remove(a)

        if len(performances) > 0:
            try:
                self.perform()
                on = False
                self.performance_runner(random.choice(performances))
            except:
                pass

        if self.state == 'analysis' and len(analysis_performances):
            # Run performances explicitly in the analysis state (Only testing performances)
            on = False
            self.performance_runner(random.choice(analysis_performances))
        if on:
            self.speech_pub.publish(msg)

    def performances_cb(self, msg):
        if msg.event == 'running':
            try:
                # Only go to performance state if robot is in interaction state
                self.perform()
            except:
                pass
        if msg.event == 'finished' \
                or msg.event == 'idle':
            if self.after_performance:
                self.after_performance()
                self.after_performance = None
            else:
                try:
                    # Return to interacting only if state was performing
                    self.interact()
                except:
                    pass


    def do_wake_up(self):
        assert (self.state == 'sleeping')
        self.btree_pub.publish(String("btree_on"))
        self.after_performance = self.to_interacting
        # Start performance before triggerring state change so soma state will be sinced with performance
        self.performance_runner('shared/wakeup')

    @staticmethod
    def _get_soma(name, magnitude):
        """ Speech"""
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

    def find_performance_by_speech(self, speech):
        """ Finds performances which one of keyword matches"""
        performances = []
        for performance, keywords in self.get_keywords().items():
            if self.performance_keyword_match(keywords, speech):
                performances.append(performance)
        return performances

    def get_keywords(self, performances=None, keywords=None, path='.'):
        if performances is None:
            performances = rospy.get_param(os.path.join('/', rospy.get_param('/robot_name'), 'webui/performances'))
            keywords = {}

        if 'properties' in performances and 'keywords' in performances['properties']:
            keywords[path] = performances['properties']['keywords']

        for key, value in performances.items():
            if key != 'properties':
                self.get_keywords(performances[key], keywords, os.path.join(path, key).strip('./'))

        return keywords

    def enable_blinking(self, enabled=True):
        try:
            self.blender_param("bpy.data.scenes[\"Scene\"].actuators.ACT_blink_randomly.HEAD_PARAM_enabled",
                               str(enabled))
            self.blender_param("bpy.data.scenes[\"Scene\"].actuators.ACT_saccade.HEAD_PARAM_enabled",
                               str(enabled))
        except rospy.ServiceException:
            pass

    def on_enter_opencog(self):
        self.btree_pub.publish(String("opencog_on"))
        self.set_chatbot_enabled(False)

    def on_exit_opencog(self):
        self.btree_pub.publish(String("opencog_off"))
        self.set_chatbot_enabled(True)


    @staticmethod
    def check_keywords(keywords, input):
        for k in keywords:
            if k.lower() in input.lower():
                return True
        return False

    @staticmethod
    def performance_keyword_match(keywords, input):
        for keyword in keywords:
            if not keyword:
                continue
            # Currently only simple matching
            if re.search(r"\b{}\b".format(keyword), input, flags=re.IGNORECASE):
                return True
        return False

    def config_cb(self, config, level=0):
        self.config = config
        return config

    def btree_cb(self, msg):
        """
        Keeps track if behavior tree active.
        Sets ROS param accordingly
        :param msg: String
        :return:
        """
        if msg.data == "btree_on":
            rospy.set_param("/behavior_enabled", True)
        if msg.data == "btree_off":
            rospy.set_param("/behavior_enabled", False)


    def set_keep_alive(self, keep_alive=True):
        magnitude = 1.0 if keep_alive else 0.0
        self.soma_pub.publish(self._get_soma('normal', magnitude))
        self.soma_pub.publish(self._get_soma('breathing', magnitude))
        self.soma_pub.publish(self._get_soma('normal-saccades', magnitude))

    def on_enter_analysis(self):
        self.enable_blinking(False)
        self.set_keep_alive(False)
        self.behavior_paused = rospy.get_param("/behavior_enabled", True)
        if self.behavior_paused:
            self.btree_pub.publish(String("btree_off"))
        if self.is_chatbot_enabled():
            self.set_chatbot_enabled(False)
            self.chatbot_paused = True

        # Reset head position
        self.look_pub.publish(Target(1, 0, 0, 0.3))
        self.gaze_pub.publish(Target(1, 0, 0, 0.3))

    def on_exit_analysis(self):
        self.enable_blinking(True)
        self.set_keep_alive(True)
        if self.behavior_paused:
            self.behavior_paused = False
            self.btree_pub.publish(String("btree_on"))
        if self.chatbot_paused:
            self.chatbot_paused = False
            self.set_chatbot_enabled(True)

    @staticmethod
    def set_chatbot_enabled(enabled=True):
        try:
            cl = dynamic_reconfigure.client.Client('chatbot', timeout=0.1)
            cl.update_configuration({"enable": enabled})
            cl.close()
        except:
            pass

    @staticmethod
    def is_chatbot_enabled():
        try:
            cl = dynamic_reconfigure.client.Client('chatbot', timeout=0.1)
            config = cl.get_configuration()
            return config['enable']
        except:
            return False

if __name__ == '__main__':
    WholeShow()
    rospy.spin()
