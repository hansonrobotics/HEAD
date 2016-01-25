#!/usr/bin/env python

# Nodes factory
import pprint

class Node(object):
    # Create new Node from JSON
    @classmethod
    def createNode(cls, data, start_time = 0):

        for s_cls in cls.__subclasses__():
            if data['name'] == s_cls.__name__:
                node =  s_cls(data, start_time)
                if start_time > node.start_time:
                    # Start time should be before or on node starting
                    node.finished = True
        print "Wrong node description"

    def __init__(self, data, start_time=0):
        self.data = data
        self.duration = 0
        self.start_time = 0
        self.started = False
        self.finished = False

    # By default end time is started + duration for every node
    def end_time(self):
        return self.start_time + self.duration

    # Manages node states. Currently start, finish is implemented
    # TODO make sure to allow node publishing pause and stop
    def run(self, run_time):
        # ignore the finished nodes
        if self.finished:
            return False
        if self.started:
            # Time to finish:
            if run_time >= self.end_time():
                self.stop()
            else:
                self.cont()
        else:
            if run_time > self.started:
                self.start()

    def __str__(self):
        return pprint.pformat(self.data)

    # Method to execute if node needs to start
    def start(self):
        pass

    # Method to execute while node is stopping
    def stop(self):
        pass

    # Method to call while node is running
    def cont(self):
        pass


class Animation(Node):
    pass

class Expression(Node):
    pass



