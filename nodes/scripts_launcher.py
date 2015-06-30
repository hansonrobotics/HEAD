#!/usr/bin/env python

import rospy
from blender_api_msgs.msg import SetGesture
from std_msgs.msg import String
from time import sleep
import os


def say(phrase):
    tts.publish(phrase)
    return

def play(gesture):
    msg = SetGesture()
    msg.speed = 1
    msg.magnitude = 1
    msg.repeat = 1
    msg.name = gesture
    gestures.publish(msg)
    pass

def publish_scripts():
    try:
        scripts = os.listdir(scripts_dir)
    except:
        rospy.logwarn("Cannot read directory")
        return
    scripts = '|'.join(os.path.splitext(s)[0] for s in  scripts if os.path.splitext(s)[1] == '.py')
    scripts_pub.publish(scripts)

def execute_script(msg):
    file = os.path.join(scripts_dir, msg.data+'.py')
    if not os.path.exists(file):
        rospy.logwarn("Script does not exist")
        return
    try:
        execfile(file)
    except:
        rospy.logwarn("Error executing script")

if __name__ == '__main__':
    tts = rospy.Publisher("chatbot_responses", String, queue_size=10)
    gestures = rospy.Publisher("/blender_api/set_gesture", SetGesture, queue_size=10)
    scripts_pub = rospy.Publisher("scripts", String, queue_size=10, latch=True)
    rospy.Subscriber("execute_script", String, execute_script, queue_size=10)
    rospy.init_node('performances')
    scripts_dir = rospy.get_param('~scripts_dir')
    print scripts_dir
    r= rospy.Rate(1.0)
    while not rospy.is_shutdown():
        publish_scripts()
        r.sleep()
