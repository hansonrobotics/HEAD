#!/usr/bin/env python

import unittest
import rospy
import os
import sys
import time
import glob

import rospkg
import roslaunch
import rostopic
import rosbag
import rosnode
from roslaunch import core
from testing_tools import (wait_for, wait_for_message, wait_for_messages,
                            startxvfb, stopxvfb, capture_screen,
                            run_shell_cmd, add_text_to_video,
                            concatenate_videos, play_rosbag,
                            rosbag_msg_generator, get_rosbag_file)
from blender_api_msgs.msg import *
from genpy import Duration

from roslaunch import nodeprocess
nodeprocess._TIMEOUT_SIGINT = 2
nodeprocess._TIMEOUT_SIGTERM = 1

CWD = os.path.abspath(os.path.dirname(__file__))
PKG = 'blender_api_msgs'

def parse_msg(msg):
    return eval(msg.split(':')[1].strip())

class BlenderAPITest(unittest.TestCase):

    def setUp(self):
        blender_api_path = os.path.join(
            rospkg.RosPack().get_path('blender_api_msgs'), '../blender_api')
        config = roslaunch.config.ROSLaunchConfig()
        config.add_node(
            core.Node(
                package='blender_api_msgs', node_type='blender',
                args='-y %s/Eva.blend -P %s/autostart.py' % (
                        blender_api_path, blender_api_path),
                name='blender_api')
            )
        self.runner = roslaunch.launch.ROSLaunchRunner(
            self.run_id, config, is_rostest=True)
        self.runner.launch()

        for node in config.nodes:
            wait_for('%s/%s' % (node.namespace, node.name))

        time.sleep(5) # Wait for blender rendering done

    def tearDown(self):
        self.runner.stop()

    @classmethod
    def setUpClass(self):
        self.run_id = 'test_blender_api'
        self.output_dir = '%s/output_video' % CWD
        if not os.path.isdir(self.output_dir):
            os.makedirs(self.output_dir)
        self.display = os.environ.get('DISPLAY', ':0')
        if self.display != ':0':
            startxvfb(self.display, '1920x1080x24')

    @classmethod
    def tearDownClass(self):
        if self.display != ':0':
            stopxvfb(self.display)
        if not os.path.isfile('%s/all.avi' % self.output_dir):
            videos = glob.glob('%s/*.avi' % self.output_dir)
            videos = [f for f in videos if not f.endswith('all.avi')]
            if len(videos) > 1:
                ofile = '%s/all.avi' % self.output_dir
                concatenate_videos(videos, ofile, False)


    def test_emotion_state(self):
        available_emotions = parse_msg(run_shell_cmd(
            'rostopic echo -n1 /blender_api/available_emotion_states', True))
        pub, msg_class = rostopic.create_publisher(
            '/blender_api/set_emotion_state',
            'blender_api_msgs/EmotionState', True)
        timeout = 2
        videos = []
        for emotion in available_emotions:
            video =  '%s/emotion-%s.avi' % (self.output_dir, emotion)
            with capture_screen(video, timeout):
                pub.publish(msg_class(emotion, 1, Duration(1, 0)))
            add_text_to_video(video)
            videos.append(video)
        ofile = '%s/emotions.avi' % self.output_dir
        concatenate_videos(videos, ofile, True)
        pub.unregister()

    def test_gesture(self):
        available_gestures = parse_msg(run_shell_cmd(
            'rostopic echo -n1 /blender_api/available_gestures', True))
        pub, msg_class = rostopic.create_publisher(
            '/blender_api/set_gesture',
            'blender_api_msgs/SetGesture', True)
        timeout = 2
        videos = []
        for gesture in available_gestures:
            if gesture == 'all': continue
            video =  '%s/gesture-%s.avi' % (self.output_dir, gesture)
            with capture_screen(video, timeout):
                pub.publish(msg_class(gesture, 1, 1, 1))
            add_text_to_video(video)
            videos.append(video)
        ofile = '%s/gestures.avi' % self.output_dir
        concatenate_videos(videos, ofile, True)
        pub.unregister()

    def test_viseme(self):
        available_visemes = parse_msg(run_shell_cmd(
            'rostopic echo -n1 /blender_api/available_visemes', True))
        pub, msg_class = rostopic.create_publisher(
            '/blender_api/queue_viseme',
            'blender_api_msgs/Viseme', True)
        timeout = 2
        videos = []
        for viseme in available_visemes:
            if 'old' in viseme: continue
            video =  '%s/viseme-%s.avi' % (self.output_dir, viseme)
            with capture_screen(video, timeout):
                pub.publish(msg_class(
                        viseme, Duration(0, 0), Duration(0, 5*1e8), 0.1, 0.8, 1))
            add_text_to_video(video)
            videos.append(video)
        ofile = '%s/viseme.avi' % self.output_dir
        concatenate_videos(videos, ofile, True)
        pub.unregister()

    def test_gaze_target(self):
        pub, msg_class = rostopic.create_publisher(
            '/blender_api/set_gaze_target',
            'blender_api_msgs/Target', True)
        timeout = 1
        targets = {
            'center': (1,0,0),
            'right':(0,1,0),
            'left':(0,-1,0),
            'up':(1,0,0.5),
            'down':(1,0,-0.5)}
        videos = []
        for name in ['right', 'up', 'left', 'down', 'center']:
            video = '%s/gaze-%s.avi' % (self.output_dir, name)
            with capture_screen(video, timeout):
                pub.publish(msg_class(*targets[name]))
            add_text_to_video(video)
            videos.append(video)
        ofile = '%s/gaze.avi' % self.output_dir
        concatenate_videos(videos, ofile, True)
        pub.unregister()

    def test_face_target(self):
        pub, msg_class = rostopic.create_publisher(
            '/blender_api/set_face_target',
            'blender_api_msgs/Target', True)
        timeout = 2
        targets = {
            'center': (1,0,0),
            'right':(0,1,0),
            'left':(0,-1,0),
            'up':(1,0,0.5),
            'down':(1,0,-0.5)}
        videos = []
        for name in ['right', 'up', 'left', 'down', 'center']:
            video = '%s/face-%s.avi' % (self.output_dir, name)
            with capture_screen(video, timeout):
                pub.publish(msg_class(*targets[name]))
            add_text_to_video(video)
            videos.append(video)
        ofile = '%s/face.avi' % self.output_dir
        concatenate_videos(videos, ofile, True)
        pub.unregister()

    def test_long_viseme(self):
        filename = get_rosbag_file('long_viseme')
        #job = play_rosbag(filename)
        #job.join()
        pub, msg_class = rostopic.create_publisher(
            '/blender_api/queue_viseme', 'blender_api_msgs/Viseme', True)
        bag = rosbag.Bag(filename)
        duration = bag.get_end_time() - bag.get_start_time()
        fps = bag.get_message_count() / float(duration)
        wait = 1.0/fps/10 # 10 times faster than the speed of the msg recoreded
        for topic, msg in rosbag_msg_generator(filename):
            pub.publish(msg)
            time.sleep(wait)
        # Test if blender is still alive
        self.assertIn('/blender_api', rosnode.get_node_names())
        self.assertIn('blender_api-1', self.runner.pm.get_active_names())


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'blender_api', BlenderAPITest)

