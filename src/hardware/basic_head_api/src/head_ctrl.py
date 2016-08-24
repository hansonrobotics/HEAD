#!/usr/bin/env python
import os
import yaml
import rospy
from collections import OrderedDict
import FaceExpr
from animation import Animation
from playback import Playback
from ros_pololu.msg import MotorCommand
from basic_head_api.srv import *
from basic_head_api.msg import *
from pau2motors.msg import pau
from geometry_msgs.msg import Quaternion
from pi_face_tracker.msg import Faces
import Quat
from std_msgs.msg import String
from std_msgs.msg import Float64
import time


def to_dict(list, key):
    result = {}
    for entry in list:
        result[entry[key]] = entry
    return result

class PauCtrl:

    def point_head(self, req):
        msg = pau()
        msg.m_headRotation = Quaternion(
            *Quat.Quat.fromInYZX(req.roll, -req.yaw, -req.pitch).params
        )
        self.pub_neck.publish(msg)

    def __init__(self):
        # PAU commands will be sent to these publishers
        self.pub_neck = rospy.Publisher("cmd_neck_pau", pau, queue_size=30)


class SpecificRobotCtrl:

    def valid_exprs(self):
        return {"exprnames": [x for x in self.faces.keys() if x[:4] != "vis_"]}

    def make_face(self, exprname, intensity=1):
        for cmd in self.faces[exprname].new_msgs(intensity):
            if exprname[:4] == 'vis_':
                cmd.speed = 0.2
                cmd.acceleration = 0.1
            self.publisher(cmd)

    def publisher(self, cmd):
        (cmd.joint_name, pubid) = cmd.joint_name.split('@')
        # Maximum speed for visimes

        # Dynamixel commands only sends position
        if self.publishers[pubid].type == 'std_msgs/Float64':
            self.publishers[pubid].publish(cmd.position)
        else:
            self.publishers[pubid].publish(cmd)


    def play_animation(self, animation, fps):
        self.playback.play(self.animations[animation],fps, animation)

    def animation_length(self, req):
        if req.name in self.animations.keys():
            return AnimationLengthResponse(self.animations[req.name].total)
        else:
            return AnimationLengthResponse(0)

    def __init__(self):
        # Wait for motors to be loaded in param server
        time.sleep(3)
        robot_name = rospy.get_param('/robot_name')
        if robot_name:
            motors = rospy.get_param('/' + robot_name + '/motors')
        else:
            motors = rospy.get_param('motors')
        expressions = rospy.get_param('expressions',{})
        expressions = OrderedDict((v.keys()[0],v.values()[0]) for k,v in enumerate(expressions))
        #Expressions to motors mapping
        self.faces = FaceExpr.FaceExprMotors.from_expr_yaml(expressions, to_dict(motors, "name"))
        # Animation objects
        animations = rospy.get_param('animations',{})
        animations = OrderedDict((v.keys()[0],v.values()[0]) for k,v in enumerate(animations))
        self.animations = Animation.from_yaml(animations)
        # Motor commands will be sent to this publisher.
        self.publishers = {}
        # Prevents from playing two animations with same prefix
        # For example Left and Right arms can be played at same time but not two animations for same arm.
        # loaded from param server in robot config
        self.animationChannels = rospy.get_param('kf_anim_channels', [])
        self.playback = Playback(to_dict(motors, "name"), self.publisher, self.animationChannels)
        # Subscribe motor topics based on their type
        for m in motors:
            if not m['topic'] in self.publishers.keys():
                # Pololu motor if motor_id is specified
                if m['hardware'] == 'pololu':
                    self.publishers[m['topic']] = rospy.Publisher(m['topic']+"/command",MotorCommand, queue_size=30)
                else:
                    self.publishers[m['topic']] = rospy.Publisher(m['topic']+"_controller/command",Float64, queue_size=30)


class HeadCtrl:

    def valid_face_exprs(self,req):
        return self.robot_ctrl.valid_exprs()

    def face_request(self, req):
        self.robot_ctrl.make_face(
            req.exprname,
            req.intensity
        )

    def animation_request(self, req):
        self.robot_ctrl.play_animation(
            req.animation,
            req.fps
        )

    def animation_length(self, req):
        return self.robot_ctrl.animation_length(req)

    def __init__(self):
        rospy.init_node('head_ctrl')
        # Deprecated. WebUI should send direct commands
        self.pau_ctrl = PauCtrl()
        rospy.Subscriber("point_head", PointHead, self.pau_ctrl.point_head)

        # Robot Control
        self.robot_ctrl= SpecificRobotCtrl()
        rospy.Service("valid_face_exprs", ValidFaceExprs, self.valid_face_exprs)
        rospy.Subscriber("make_face_expr", MakeFaceExpr, self.face_request)
        # Animations
        rospy.Subscriber("play_animation", PlayAnimation, self.animation_request)
        rospy.Service("animation_length", AnimationLength, self.animation_length)

if __name__ == '__main__':
    HeadCtrl()
    rospy.loginfo("Started")
    rospy.spin()
