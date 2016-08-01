#!/usr/bin/env python

import random
import rospy
from std_msgs.msg import String
import dynamic_reconfigure.client
from dynamic_reconfigure.server import Server
from basic_head_api.cfg import ArmsConfig
from basic_head_api.msg import PlayAnimation
from basic_head_api.srv import *

class ArmsAlive:
    def __init__(self):
        rospy.init_node('arms_alive')
        rospy.wait_for_service('animation_length', timeout=3)
        rospy.Subscriber('speech_events', self.tts_status)
        self.enabled = rospy.get_param('arms_alive')
        self.arm_movements = rospy.get_param('arm_movements')
        self.anim_length = rospy.ServiceProxy('animation_length', AnimationLength)
        self.play = rospy.Publisher('play_animation', PlayAnimation, queue_size=5)
        # Storing animation lengths
        self.lengths = {}

    # Gets lengths of each animation
    def parse_lengths(self):
        for arm, params in self.arm_movements.iteritems():
            for a in params['animations']:
                res = self.anim_length(a)
                self.lengths[a] = res.frames

    # monitor status from TTS. Reacts to duration messages
    def tts_status(self, msg):
        if not self.enabled:
            return
        data = msg.data
        # Gets tts duration
        try:
            data.index("duration:")
            duration = float(data[len("duration:"):])
        except ValueError:
            return
        self.playAnimations(duration)

    # Plays animation based duration:
    def playAnimations(self, duration):
        # separate random number for each arm
        for arm in self.arm_movements.values():
            r = random.random()
            fps = False
            if duration < 2 and r < arm['probabilities'][0]:
                animation,fps = self.pickAnimation(arm,duration)
            if 2 <= duration < 4 and r < arm['probabilities'][1]:
                animation,fps = self.pickAnimation(arm,duration)
            if 4 <= duration and r < arm['probabilities'][2]:
                animation,fps = self.pickAnimation(arm,duration)
            if fps:
                self.play(PlayAnimation(animation,fps))

    # Picks animations
    # Currently random animation and FPS
    # TODO make sure it can pick based on duratiuon required and plan multiple for really long sentences
    def pickAnimation(self, arm, duration):
        animation = random.choice(arm['animations'])
        fps = random.randrange(30,49)
        return (animation,fps)

    # Allows enable/disaable arms_alive gestures
    def reconfig(self, config, level):
        self.enabled = config.arms_alive
        return config





if __name__ == '__main__':
    armsNode = ArmsAlive()
    Server(ArmsConfig, armsNode.reconfig)
    rospy.spin()
