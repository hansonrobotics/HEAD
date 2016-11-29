#!/usr/bin/env python
__author__ = 'ss'

import rospy
from pi_face_tracker.msg import Faces
from std_msgs.msg import String
from ros_pololu.msg import MotorCommand
import math


class EyeContact:
    NODE_NAME = 'eye_contact'

    def __init__(self):
        self.command_publisher = rospy.Publisher('/sophia/safe/head/command', MotorCommand, queue_size=10)
        self.face_subscriber = rospy.Subscriber("/camera/face_locations", Faces, self.callback)
        self.degree_hor = 0
        self.degree_ver = 0
        self.command_msg = MotorCommand()
        self.command_msg.position = 0
        self.command_msg.speed = 0.05
        self.command_msg.acceleration = 0.2

    def callback(self, data):
        if len(data.faces) == 1:
            self.set_eye_degree_hor_relative((data.faces[0].point.y - 0.05) * 20)
            self.set_eye_degree_ver_relative(-(data.faces[0].point.z - 0.00) * 40)

    def set_eye_degree_hor_relative(self, degree):
        self.set_eye_degree_hor(self.degree_hor + degree)

    def set_eye_degree_hor(self, degree):
        print "Set hor_degree: ", degree
        self.degree_hor = max(min(45, degree), -45)
        degree_hor_r = max(min(45, degree - 15), -45)
        degree_hor_l = max(min(45, degree), -45)
        self.command_msg.position = math.radians(degree_hor_r)
        self.command_msg.joint_name = "EyeTurn-R"
        self.command_publisher.publish(self.command_msg)
        self.command_msg.position = math.radians(degree_hor_l)
        self.command_msg.joint_name = "EyeTurn-L"
        self.command_publisher.publish(self.command_msg)

    def set_eye_degree_ver_relative(self, degree):
        self.set_eye_degree_ver(self.degree_ver + degree)

    def set_eye_degree_ver(self, degree):
        print "Set ver_degree: ", degree
        self.degree_ver = max(min(45, degree - 5), -45)
        self.command_msg.position = math.radians(self.degree_ver)
        self.command_msg.joint_name = "Eyes-Up-Down"
        self.command_publisher.publish(self.command_msg)


if __name__ == '__main__':
    rospy.loginfo("Starting " + EyeContact.NODE_NAME)
    eye_contact = EyeContact()
    rospy.init_node(EyeContact.NODE_NAME, anonymous=False)
    try:
        eye_contact.set_eye_degree_hor(0)
        eye_contact.set_eye_degree_ver(0)
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Stopping " + EyeContact.NODE_NAME)
