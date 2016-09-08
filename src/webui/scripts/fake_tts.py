#!/usr/bin/env python

import rospy
from chatbot.msg import ChatMessage
from std_msgs.msg import String

def cb(msg):
    pub.publish(msg.data)

rospy.init_node('fake_tts')

rospy.Subscriber('chatbot_responses', String, cb)
pub = rospy.Publisher('tts', String, queue_size=1)


while not rospy.is_shutdown():
    rospy.spin()
