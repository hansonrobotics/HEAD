#!/usr/bin/env python

import rospy
from blender_api_msgs.msg import SetGesture
from blender_api_msgs.msg import EmotionState
from std_msgs.msg import String
from time import sleep
import os


def say(phrase):
    tts_pub.publish(phrase)
    return

def play(gesture, speed=1, magnitude=1, repeat=1):
    msg = SetGesture()
    msg.speed = speed
    msg.magnitude = magnitude
    msg.repeat = repeat
    msg.name = gesture
    gestures_pub.publish(msg)

def emotion(emo, magnitude=1, duration=1):
    msg = EmotionState()
    msg.name = emo
    msg.magnitude = magnitude
    msg.duration = rospy.Duration.from_sec(duration)
    emotions_pub.publish(msg)


def publish_scripts():
    try:
        scripts = os.listdir(scripts_dir)
    except:
        rospy.logwarn("Cannot read directory")
        return
    scripts.sort()
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
    tts_pub = rospy.Publisher("chatbot_responses", String, queue_size=10)
    gestures_pub = rospy.Publisher("/blender_api/set_gesture", SetGesture, queue_size=10)
    emotions_pub = rospy.Publisher("/blender_api/set_emotion_state", EmotionState, queue_size=10)
    scripts_pub = rospy.Publisher("scripts", String, queue_size=10, latch=True)
    rospy.Subscriber("execute_script", String, execute_script, queue_size=10)
    rospy.init_node('performances')
    scripts_dir = rospy.get_param('~scripts_dir')
    r= rospy.Rate(1.0)
    while not rospy.is_shutdown():
        publish_scripts()
        r.sleep()
