#!/usr/bin/env python

import rospy
from ros_pololu.msg import *
from std_msgs.msg import Float64


import time


class Safety():
    def __init__(self):
        self.topics = []
        # Wait for motors to be loaded in param server
        time.sleep(3)
        motors = rospy.get_param('motors')
        # subscribe
        self.motor_positions = {}
        self.subscribers = {}
        self.publishers = {}
        # Create proxy topics and subscribers
        for m in motors:
            if not m['topic'] in self.publishers.keys():
                # Pololu motor if motor_id is specified
                print m['topic']
                if 'motor_id' in m:
                    self.publishers[m['topic']] = rospy.Publisher("safe/"+m['topic']+"/command",MotorCommand, queue_size=30)
                    self.subscribers[m['topic']] = rospy.Subscriber(m['topic']+"/command", MotorCommand,
                                        lambda msg,topic=m['topic']: self.callback(topic, False, msg))
                else:
                    self.publishers[m['topic']] = rospy.Publisher("safe/"+m['topic']+"_controller/command",Float64, queue_size=30)
                    self.subscribers[m['topic']] = rospy.Subscriber(m['topic']+"_controller/command", Float64,
                                        lambda msg,topic=m['topic']: self.callback(topic, True, msg))


    def callback(self, topic_name, dynamixel, msg):
        # Republish message to safe topic
        self.publishers[topic_name].publish(msg)

if __name__ == '__main__':
    rospy.init_node('motors_safety')
    MS = Safety()
    rospy.spin()
