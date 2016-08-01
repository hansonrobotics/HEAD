import random
import rospy
from std_msgs.msg import String
import dynamic_reconfigure.client
from dynamic_reconfigure.server import Server
from tts.cfg import TTSConfig

class ArmsAlive:
    def __init__(self):
        rospy.init_node('arms_alive')
        rospy.wait_for_service('animation_length', timeout=3)
        rospy.Subscriber('speech_events')


    def tts_status(self, msg):
        pass

if __name__ == '__main__':
    ArmsAlive()
    rospy.loginfo("Started")
    rospy.spin()
