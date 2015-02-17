#!/usr/bin/env python
import os
import yaml
import rospy
from collections import OrderedDict
import FaceExpr
from ros_pololu_servo.msg import MotorCommand
from basic_head_api.srv import *
from basic_head_api.msg import *
from pau2motors.msg import pau
from geometry_msgs.msg import Quaternion
from pi_face_tracker.msg import Faces
import Utils
from std_msgs.msg import String
from std_msgs.msg import Float64
import copy


def to_dict(list, key):
  result = {}
  for entry in list:
    result[entry[key]] = entry
  return result

class PauCtrl:

  def point_head(self, req):
    rospy.loginfo(
      "Point head (yaw: %s, pitch: %s, roll: %s)",
      req.yaw, req.pitch, req.roll
    )
    msg = pau()
    msg.m_headRotation = Quaternion(
      *Utils.Quat.fromInYZX(req.roll, -req.yaw, -req.pitch).params
    )
    self.pub_neck.publish(msg)
  
  def __init__(self):
    # PAU commands will be sent to these publishers
    self.pub_neck = rospy.Publisher("cmd_neck_pau", pau, queue_size=30)


class SpecificRobotCtrl:

  def valid_exprs(self):
    rospy.loginfo("Valid face expressions request.")
    return {"exprnames": self.faces.keys()}

  def make_face(self, exprname, intensity=1):
    rospy.loginfo("Face request: %s of %s", intensity, exprname)
    for cmd in self.faces[exprname].new_msgs(intensity):
      (cmd.joint_name, pubid) = cmd.joint_name.split('@')
      rospy.loginfo("Pub id: %s", pubid)
      # Dynamixel commands only sends position
      if self.publishers[pubid].type == 'std_msgs/Float64':
        self.publishers[pubid].publish(cmd.position)
      else:
        self.publishers[pubid].publish(cmd)

  def __init__(self):
    motors = rospy.get_param('motors')
    expressions = rospy.get_param('expressions')
    #Expressions to motors mapping
    self.faces = FaceExpr.FaceExprMotors.from_expr_yaml(expressions, to_dict(motors, "name"))
    # Motor commands will be sent to this publisher.
    self.publishers = {}
    # Subscribe motor topics based on their type
    for m in motors:
      if not m['topic'] in self.publishers.keys():
        # Pololu motor if motor_id is specified
        if 'motor_id' in m: 
          self.publishers[m['topic']] = rospy.Publisher(m['topic']+"/command",MotorCommand, queue_size=30)
        else:
          self.publishers[m['topic']] = rospy.Publisher(m['topic']+"/command",Float64, queue_size=30)


class HeadCtrl:

  def valid_face_exprs(self,req):
    return self.robot_ctrl.valid_exprs()

  def face_request(self, req):
    self.robot_ctrl.make_face(
      req.exprname,
      req.intensity
    )

  def __init__(self):
    rospy.init_node('head_ctrl')
    # Deprecated. WebUI should send direct commands
    self.pau_ctrl = PauCtrl()
    rospy.Subscriber("point_head", PointHead, self.pau_ctrl.point_head)

    # Robot Control  
    self.robot_ctrl= SpecificRobotCtrl()
    rospy.Service("valid_face_exprs", ValidFaceExprs, self.valid_face_exprs)
    rospy.Subscriber("make_face_expr", MakeFaceExpr, self.face_request)

if __name__ == '__main__':
    HeadCtrl()
    rospy.loginfo("Started")
    rospy.spin()
