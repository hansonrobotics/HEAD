#!/usr/bin/env python

# Nodes factory
import pprint
import rospy
from std_msgs.msg import String
from blender_api_msgs.msg import SetGesture, EmotionState, Target
from basic_head_api.msg import MakeFaceExpr
from topic_tools.srv import MuxSelect

class Node(object):
    # Create new Node from JSON
    @classmethod
    def createNode(cls, data, runner, start_time=0):
        for s_cls in cls.__subclasses__():
            if data['name'] == s_cls.__name__:
                node =  s_cls(data, runner)
                if start_time > node.start_time:
                    # Start time should be before or on node starting
                    node.finished = True
                return node
        print "Wrong node description"

    def __init__(self, data, runner):
        self.data = data
        self.duration = data['duration']
        self.start_time = data['start_time']
        self.started = False
        self.finished = False
        # Node runner for accessing ROS topics and method
        # TODO make ROS topics and services singletons class for shared use.
        self.runner = runner

    # By default end time is started + duration for every node
    def end_time(self):
        return self.start_time + self.duration

    # Manages node states. Currently start, finish is implemented.
    # Returns True if its active, and False if its inactive.
    # TODO make sure to allow node publishing pause and stop
    def run(self, run_time):
        # ignore the finished nodes
        if self.finished:
            return False
        if self.started:
            # Time to finish:
            if run_time >= self.end_time():
                self.stop(run_time)
                self.finished = True
                return False
            else:
                self.cont(run_time)
        else:
            if run_time > self.start_time:
                self.start(run_time)
                self.started = True
        return True

    def __str__(self):
        return pprint.pformat(self.data)

    # Method to execute if node needs to start
    def start(self, run_time):
        pass

    # Method to execute while node is stopping
    def stop(self, run_time):
        pass

    # Method to call while node is running
    def cont(self, run_time):
        pass


class speech(Node):

    def start(self, run_time):
        self.say(self.data['text'], self.data['lang'])

    def say(self, text, lang):
        if lang not in ['en', 'zh']:
            lang = 'default'
        self.runner.topics['tts'][lang].publish(String(text))


class gesture(Node):

    def start(self, run_time):
        self.runner.topics['gesture'].publish(
                SetGesture(self.data['gesture'], 1, float(self.data['speed']), float(self.data['magnitude'])))


class emotion(Node):

    def start(self, run_time):
        self.runner.topics['emotion'].publish(
                EmotionState(self.data['emotion'], float(self.data['magnitude']),
                             rospy.Duration.from_sec(self.data['duration'])))


class look_at(Node):

    def start(self, run_time):
        self.runner.topics['look_at'].publish(Target(self.data['x'], self.data['y'], self.data['z']))


class gaze_at(Node):

    def start(self, run_time):
        self.runner.topics['gaze_at'].publish(Target(self.data['x'], self.data['y'], self.data['z']))


class interaction(Node):

    def start(self, run_time):
        self.runner.topics['interaction'].publish(String('btree_on'))

    def stop(self, run_time):
        self.runner.topics['interaction'].publish(String('btree_off'))


class expression(Node):
    def __init__(self, data, runner, start_time=0):
        Node.__init__(self, data, runner, start_time)
        self.shown = False

    def start(self, run_time):
        self.runner.services['head_pau_mux']({'topic': "/" + self.runner.robot_name + "/no_pau"})
        self.shown = False

    def cont(self, run_time):
        # Publish expression message after some delay once node is started
        if (not self.shown) and (run_time > self.start_time + 0.05):
            self.shown = True
            self.runner.topics['expression'].publish(MakeFaceExpr(self.data['expression'], float(self.data['magnitude'])))

    def stop(self, run_time):
        self.runner.services['head_pau_mux']({'topic': "/" + self.runner.robot_name + "/head_pau"})


class pause(Node):

    def start(self, run_time):
        self.runner.make_pause()
