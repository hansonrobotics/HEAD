#!/usr/bin/env python

import rospy
from ros_pololu.msg import MotorCommand
from std_msgs.msg import Float64


import time


class Safety():
    def __init__(self):
        self.topics = []
        # Wait for motors to be loaded in param server
        time.sleep(3)
        motors = rospy.get_param('motors')
        self.rules = rospy.get_param('safety_rules', {})
        # subscribe
        self.motor_positions = {}
        self.subscribers = {}
        self.publishers = {}
        self.motors = {}
        # Create proxy topics and subscribers
        for m in motors:
            self.motor_positions[m['name']] = m['default']
            self.motors[m['name']] = m
            if not m['topic'] in self.publishers.keys():
                # Pololu motor if motor_id is specified
                if 'motor_id' in m:
                    self.publishers[m['topic']] = rospy.Publisher("safe/"+m['topic']+"/command",MotorCommand, queue_size=30)
                    self.subscribers[m['topic']] = rospy.Subscriber(m['topic']+"/command", MotorCommand,
                                    lambda msg,m=m: self.callback(m, False, msg))
                else:
                    self.publishers[m['topic']] = rospy.Publisher("safe/"+m['topic']+"_controller/command",Float64, queue_size=30)
                    self.subscribers[m['topic']] = rospy.Subscriber(m['topic']+"_controller/command", Float64,
                                        lambda msg, m=m: self.callback(m, True, msg))

    def callback(self, motor, dynamixel, msg):
        # Republish message to safe topic
        mname = motor['name']
        if mname in self.rules.keys():
            self.process_rules(mname, dynamixel, msg)
        if dynamixel:
            v = msg.data
        else:
            v = msg.position
        self.motor_positions[mname] = v
        self.publishers[motor['topic']].publish(msg)

    def process_rules(self, motor, dynamixel, msg):
        if dynamixel:
            v = msg.data
        else:
            v = msg.position
        rules = self.rules[motor]
        for r in rules:
            if r['type'] == 'prevent':
                v = self.rule_prevent(motor, v, r)
        if dynamixel:
            msg.data = v
        else:
            msg.position = v
    #Prevents motor going over its safe range if the other oposite motors are set to different directions
    def rule_prevent(self, motor, v, rule):
        # Check if its over extreme
        if self.get_relative_pos(motor, rule['direction'], v) <= rule['extreme']:
            return v
        # Check dependency
        if self.get_relative_pos(rule['depends'], rule['dep_dir'], self.motor_positions[rule['depends']]) > rule['dep_extreme']:
            # Preventing form getting over limit. sets motor position to limit before extreme
            return self.get_abs_pos(motor, rule['direction'], rule['extreme'])
        return v

    # Gets absolute position from relative between the neutral and extreme in given direction
    def get_abs_pos(self, motor, direction, v):
        extreme = self.motors[motor][direction]
        return self.motors[motor]['default'] + (extreme - self.motors[motor]['default']) * v
    # Gets relative position from abs to the direction of given extreme

    def get_relative_pos(self, motor, direction, v):
        extreme = self.motors[motor][direction]
        # avoid div by 0
        if extreme == self.motors[motor]['default']:
            return 0
        return (v - self.motors[motor]['default']) / (extreme - self.motors[motor]['default'])


if __name__ == '__main__':
    rospy.init_node('motors_safety')
    MS = Safety()
    rospy.spin()
