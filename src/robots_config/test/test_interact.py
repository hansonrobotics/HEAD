#!/usr/bin/env python

import unittest
import os
import rostopic
import rospy
import roslaunch
from testing_tools.misc import capture_screen, MessageQueue, add_text_to_video, wait_for
from testing_tools.blender import set_alive
from std_msgs.msg import String
import subprocess

CWD = os.path.abspath(os.path.dirname(__file__))

class InteractTest(unittest.TestCase):

    @classmethod
    def setUpClass(self):
        self.robot_name = 'han'
        launch_file = '{}/launch/robot.launch'.format(CWD)
        cmd = 'roslaunch {} basedir:={basedir} name:={name}'.format(
            launch_file, basedir='launch', name=self.robot_name)
        self.proc = subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid)
        for node in ['chatbot_en', 'chatbot_zh', 'speech2command', 'tts']:
            wait_for(node, self.robot_name)
        wait_for('blender_api')
        rospy.wait_for_service('/blender_api/set_param')

        self.output_video = os.path.expanduser('~/.hr/test/screencast')
        if not os.path.isdir(self.output_video):
            os.makedirs(self.output_video)
        self.output_audio = os.path.expanduser('~/.hr/test/audio')
        if not os.path.isdir(self.output_audio):
            os.makedirs(self.output_audio)
        self.queue = MessageQueue()
        self.queue.subscribe(
            '/{}/chatbot_responses'.format(self.robot_name), String)

    @classmethod
    def tearDownClass(self):
        if self.proc:
            os.killpg(self.proc.pid, 2)

    def setUp(self):
        set_alive(False)
        rospy.sleep(1)

    def tearDown(self):
        set_alive(True)
        rospy.sleep(1)

    def test_emotion_cmd(self):
        duration = 6
        pub, msg_class = rostopic.create_publisher(
            '/{}/speech'.format(self.robot_name), 'chatbot/ChatMessage', True)
        for cmd in ['sad', 'happy']:
            screen_output = '%s/bl_%s.avi' % (
                self.output_video, cmd.replace(' ', '_'))
            with capture_screen(screen_output, duration):
                rospy.sleep(0.5)
                pub.publish(msg_class(cmd, 100))

    def test_gesture_cmd(self):
        duration = 2
        pub, msg_class = rostopic.create_publisher(
            '/{}/speech'.format(self.robot_name), 'chatbot/ChatMessage', True)
        for cmd in ['blink', 'nod', 'shake']:
            screen_output = '%s/bl_%s.avi' % (
                self.output_video, cmd.replace(' ', '_'))
            with capture_screen(screen_output, duration):
                rospy.sleep(0.5)
                pub.publish(msg_class(cmd, 100))

    def test_turn_eye_cmd(self):
        duration = 2
        pub, msg_class = rostopic.create_publisher(
            '/{}/speech'.format(self.robot_name), 'chatbot/ChatMessage', True)
        pub.publish(msg_class('look center', 100))
        rospy.sleep(duration)
        for cmd in ['look left', 'look right']:
            screen_output = '%s/bl_%s.avi' % (
                self.output_video, cmd.replace(' ', '_'))
            with capture_screen(screen_output, duration):
                rospy.sleep(0.5)
                pub.publish(msg_class(cmd, 100))
            pub.publish(msg_class('look center', 100))
            rospy.sleep(duration)

    def test_turn_head_cmd(self):
        duration = 2
        pub, msg_class = rostopic.create_publisher(
            '/{}/speech'.format(self.robot_name), 'chatbot/ChatMessage', True)
        pub.publish(msg_class('turn center', 100))
        rospy.sleep(duration)
        for cmd in ['turn left', 'turn right']:
            screen_output = '%s/bl_%s.avi' % (
                self.output_video, cmd.replace(' ', '_'))
            with capture_screen(screen_output, duration):
                rospy.sleep(0.5)
                pub.publish(msg_class(cmd, 100))
            pub.publish(msg_class('turn center', 100))
            rospy.sleep(duration)

    def test_chat(self):
        pub, msg_class = rostopic.create_publisher(
            '/{}/speech'.format(self.robot_name), 'chatbot/ChatMessage', True)
        self.queue.clear()
        for speech in ['hi', 'what\'s your name']:
            pub.publish(msg_class(speech, 100))
            rospy.sleep(1)
        msgs = self.queue.tolist()
        self.assertEqual(msgs[0].data, 'Hi there!')
        self.assertTrue('Soepheeyeh' in msgs[1].data)

if __name__ == '__main__':
    unittest.main()

