#!/usr/bin/env python
import os
import yaml
import rospy
from collections import OrderedDict
import FaceExpr
from ros_pololu_servo.msg import servo_pololu
from basic_head_api.srv import *
from basic_head_api.msg import *
from pau2motors.msg import pau
from geometry_msgs.msg import Quaternion
import Utils
from Blinker import Blinker

CONFIG_DIR = "config"

#Extend yaml functionality
def ordered_load(stream, Loader=yaml.Loader, object_pairs_hook=OrderedDict):
    class OrderedLoader(Loader):
        pass
    OrderedLoader.add_constructor(
        yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
        lambda loader, node: object_pairs_hook(loader.construct_pairs(node)))
    return yaml.load(stream, OrderedLoader)
yaml.ordered_load = ordered_load
#---

def read_config(configname):
  # Find the directory of this python file.
  dirname, filename = os.path.split(os.path.abspath(__file__))

  stream = open(os.path.join(dirname, CONFIG_DIR, configname), 'r')
  config = yaml.ordered_load(stream)
  stream.close()
  return config

def to_dict(list, key):
  result = {}
  for entry in list:
    result[entry[key]] = entry
  return result

class PauCtrl:

  def valid_exprs(self):
    rospy.loginfo("Valid face expressions request.")
    return {"exprnames": self.faces.keys()}

  def make_face(self, exprname, intensity=1):
    rospy.loginfo("Face request: %s of %s", intensity, exprname)

    if exprname == 'neutral' or intensity == 0:
      # Send a neutral expression, which is not saved among other expressions.
      self.pub_face.publish(FaceExpr.FaceExprPAU().new_msg(0))
    else:
      self.pub_face.publish(
        self.faces[exprname].new_msg(intensity)
      )

  def point_head(self, req):
    rospy.loginfo(
      "Point head (yaw: %s, pitch: %s, roll: %s)",
      req.yaw, req.pitch, req.roll
    )
    msg = pau()
    msg.m_headRotation = Quaternion(
      *Utils.Quat.fromInYZX(req.yaw, req.pitch, req.roll).params
    )
    self.pub_neck.publish(msg)
  
  def __init__(self, pub_face, pub_neck):
    # Dictionary of expression names mapping to FaceExprPAU instances.
    self.faces = FaceExpr.FaceExprPAU.from_expr_yaml(
      read_config("pau_exprs.yaml")
    )

    # PAU commands will be sent to these publishers
    self.pub_face = pub_face
    self.pub_neck = pub_neck


class SpecificRobotCtrl:

  def valid_exprs(self):
    rospy.loginfo("Valid %s face expressions request.", self.robotname)
    return {"exprnames": self.faces.keys()}

  def make_face(self, exprname, intensity=1):
    rospy.loginfo("Face request: %s of %s for %s", intensity, exprname, self.robotname)
    for cmd in self.faces[exprname].new_msgs(intensity):
      pubid = cmd.id // 24
      cmd.id = cmd.id % 24
      self.publishers[pubid].publish(cmd)
      self.blinker.log(cmd)

  def __init__(self, robotname, publishers):
    # Dictionary of expression names mapping to FaceExprMotors instances.
    self.faces = FaceExpr.FaceExprMotors.from_expr_yaml(
      read_config(robotname + "_exprs.yaml"),
      to_dict(read_config(robotname + "_motors.yaml"), "name")
    )

    # Motor commands will be sent to this publisher.
    self.publishers = publishers

    self.robotname = robotname
    self.blinker = Blinker(
      robotname, 
      read_config(robotname + "_motors.yaml")
      ("Eyelid-Upper_L", "Eyelid-Upper_R")
    )


class HeadCtrl:

  robot_controllers = {}

  def get_robot_ctrl(self, robotname):
    """
    Creates a new SpecificRobotCtrl instance, if the given robotname hasn't
    been accessed yet.
    """
    robotctrl = self.robot_controllers.get(robotname)
    if robotctrl == None:
      robotctrl = SpecificRobotCtrl(robotname, self.pub_pololu)
      self.robot_controllers[robotname] = robotctrl
    return robotctrl

  def valid_coupled_face_exprs(self, req):
    return self.get_robot_ctrl(req.robotname).valid_exprs()

  def coupled_face_request(self, req):
    self.get_robot_ctrl(req.robotname).make_face(
      req.expr.exprname,
      req.expr.intensity
    )

  def __init__(self):
    rospy.init_node('head_ctrl')

    # Topics and services for PAU expressions
    self.pau_ctrl = PauCtrl(
      rospy.Publisher("cmd_face_pau", pau, queue_size=2),
      rospy.Publisher("cmd_neck_pau", pau, queue_size=2)
    )
    rospy.Service("valid_face_exprs", ValidFaceExprs, 
      lambda req: self.pau_ctrl.valid_exprs()
    )
    rospy.Subscriber("make_face_expr", MakeFaceExpr,
      lambda req: self.pau_ctrl.make_face(req.exprname, req.intensity)
    )
    rospy.Subscriber("point_head", PointHead, self.pau_ctrl.point_head)

    # Topics and services for robot-specific motor-coupled expressions.
    self.pub_pololu = [None,None];
    self.pub_pololu[0] = rospy.Publisher("dmitry_face/cmd_pololu", servo_pololu, queue_size=30)
    self.pub_pololu[1] = rospy.Publisher("dmitry_eyes/cmd_pololu", servo_pololu, queue_size=30)
    rospy.Service("valid_coupled_face_exprs", ValidCoupledFaceExprs, self.valid_coupled_face_exprs)
    rospy.Subscriber("make_coupled_face_expr", MakeCoupledFaceExpr, self.coupled_face_request)

if __name__ == '__main__':
    HeadCtrl()
    rospy.loginfo("Started")
    rospy.spin()
