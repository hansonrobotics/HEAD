#!/usr/bin/env python

import unittest
import atexit
import os
import rostopic
import rospy
import roslaunch
from testing_tools.misc import capture_screen, MessageQueue, add_text_to_video, wait_for, check_if_ffmpeg_satisfied
from testing_tools.blender import set_alive
from std_msgs.msg import String, Float64
from pau2motors.msg import pau
from pau2motors.MapperFactory import Quaternion2EulerYZX
import subprocess
import math

CWD = os.path.abspath(os.path.dirname(__file__))

robot_name = 'han'
cmd = 'roslaunch {}/launch/robot.test basedir:={}/launch name:={name}'.format(
    CWD, CWD, name=robot_name)
proc = subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
def shutdown():
    if proc:
        os.killpg(proc.pid, 2)
atexit.register(shutdown)

class InteractTest(unittest.TestCase):

    @classmethod
    def setUpClass(self):
        for node in ['chatbot_en', 'chatbot_zh', 'speech2command', 'tts']:
            wait_for(node, robot_name)
        wait_for('blender_api')
        rospy.wait_for_service('/blender_api/set_param')

        self.output_video = os.path.expanduser('~/.hr/test/screencast')
        if not os.path.isdir(self.output_video):
            os.makedirs(self.output_video)
        self.output_audio = os.path.expanduser('~/.hr/test/audio')
        if not os.path.isdir(self.output_audio):
            os.makedirs(self.output_audio)
        self.queue = MessageQueue()
        rospy.sleep(10) # wait for everything to be ready. Wait for PUB/SUB connection to be stable

    def setUp(self):
        self.queue.clear()
        set_alive(False)
        rospy.sleep(1)

    def tearDown(self):
        self.queue.clear()
        set_alive(True)
        rospy.sleep(1)

    @unittest.skipUnless(
        check_if_ffmpeg_satisfied(), 'Skip because ffmpeg is not satisfied.')
    def test_emotion_cmd(self):
        duration = 6
        pub, msg_class = rostopic.create_publisher(
            '/{}/speech'.format(robot_name), 'chatbot/ChatMessage', False)
        for cmd in ['sad', 'happy']:
            screen_output = '%s/bl_%s.avi' % (
                self.output_video, cmd.replace(' ', '_'))
            with capture_screen(screen_output, duration):
                rospy.sleep(0.5)
                pub.publish(msg_class(cmd, 100))

    @unittest.skipUnless(
        check_if_ffmpeg_satisfied(), 'Skip because ffmpeg is not satisfied.')
    def test_gesture_cmd(self):
        duration = 2
        pub, msg_class = rostopic.create_publisher(
            '/{}/speech'.format(robot_name), 'chatbot/ChatMessage', False)
        for cmd in ['blink', 'nod', 'shake']:
            screen_output = '%s/bl_%s.avi' % (
                self.output_video, cmd.replace(' ', '_'))
            with capture_screen(screen_output, duration):
                rospy.sleep(0.5)
                pub.publish(msg_class(cmd, 100))

    @unittest.skipUnless(
        check_if_ffmpeg_satisfied(), 'Skip because ffmpeg is not satisfied.')
    def test_turn_eye_cmd(self):
        duration = 2
        pub, msg_class = rostopic.create_publisher(
            '/{}/speech'.format(robot_name), 'chatbot/ChatMessage', False)
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

    @unittest.skipUnless(
        check_if_ffmpeg_satisfied(), 'Skip because ffmpeg is not satisfied.')
    def test_turn_head_cmd(self):
        duration = 2
        pub, msg_class = rostopic.create_publisher(
            '/{}/speech'.format(robot_name), 'chatbot/ChatMessage', False)
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
            '/{}/speech'.format(robot_name), 'chatbot/ChatMessage', False)
        sub = self.queue.subscribe(
            '/{}/chatbot_responses'.format(robot_name), String)
        rospy.sleep(2)
        for speech in ['what\'s your name']:
            pub.publish(msg_class(speech, 100))
            rospy.sleep(1)
        msgs = self.queue.tolist()
        self.assertTrue(robot_name in msgs[0].data.lower())
        sub.unregister()

    def test_head_angle(self):
        pub, msg_class = rostopic.create_publisher(
            '/blender_api/set_face_target', 'blender_api_msgs/Target', False)
        sub = self.queue.subscribe('/blender_api/get_pau', pau)
        mapper = Quaternion2EulerYZX({'axis': 'z'}, None)
        err = 0.05
        rospy.sleep(2)

        pub.publish(msg_class(1, 1, 0))
        rospy.sleep(2)
        msgs = self.queue.tolist()
        self.assertTrue(len(msgs) > 0)
        rad = mapper.map(msgs[-1].m_headRotation)
        self.assertLess(abs(-rad-math.atan(1)), err)
        self.queue.clear()

        pub.publish(msg_class(1, -1, 0))
        rospy.sleep(2)
        msgs = self.queue.tolist()
        self.assertTrue(len(msgs) > 0)
        rad2 = mapper.map(msgs[-1].m_headRotation)
        self.assertLess(abs(rad2+rad), err)

        self.queue.clear()
        pub.publish(msg_class(2, 1, 0))
        rospy.sleep(2)
        msgs = self.queue.tolist()
        self.assertTrue(len(msgs) > 0)
        rad3 = mapper.map(msgs[-1].m_headRotation)
        self.assertLess(abs(-rad3-math.atan(0.5)), err)
        self.assertTrue(rad*rad3>0) # the same sign

        pub.publish(msg_class(1, 0, 0))
        rospy.sleep(2)
        sub.unregister()

    def test_eye_angle(self):
        pub, msg_class = rostopic.create_publisher(
            '/blender_api/set_gaze_target', 'blender_api_msgs/Target', False)
        sub = self.queue.subscribe('/blender_api/get_pau', pau)
        err = 0.05
        rospy.sleep(2)
        pub.publish(msg_class(1, 1, 0))
        rospy.sleep(2)
        msgs = self.queue.tolist()
        self.assertTrue(len(msgs) > 0)
        rad = (msgs[-1].m_eyeGazeLeftYaw+msgs[-1].m_eyeGazeRightYaw)/2
        self.assertLess(abs(-rad-math.atan(1)), err)

        pub.publish(msg_class(1, 0, 0))
        rospy.sleep(2)
        sub.unregister()

if __name__ == '__main__':
    unittest.main()

