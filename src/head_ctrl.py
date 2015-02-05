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
from Blinker import Blinker
from Blinker import RandomTimer
from std_msgs.msg import String
from threading import Timer
import copy

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
      *Utils.Quat.fromInYZX(req.roll, req.yaw, -req.pitch).params
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
      self.blinker.log(copy.deepcopy(cmd))
      (cmd.joint_name, pubid) = cmd.joint_name.split('@')
      rospy.loginfo("Pub id: %s", pubid) 
      self.publishers[pubid].publish(cmd)

  def blink(self):
    def close_lids():
      for cmd in self.blinker.new_msgs(1.0):
        (cmd.joint_name, pubid) = cmd.joint_name.split('@')
        self.publishers[pubid].publish(cmd)
    def open_lids():
      for cmd in self.blinker.reset_msgs():
        self.publishers[0].publish(cmd)
        (cmd.joint_name, pubid) = cmd.joint_name.split('@')
        self.publishers[pubid].publish(cmd)
    close_lids()
    Timer(0.1, open_lids).start()
    

  def __init__(self, robotname, publishers):
    # Dictionary of expression names mapping to FaceExprMotors instances.
    self.faces = FaceExpr.FaceExprMotors.from_expr_yaml(
      read_config(robotname + "_exprs.yaml"),
      to_dict(read_config(robotname + "_motors.yaml"), "name")
    )

    # Motor commands will be sent to this publisher.
    self.publishers = publishers

    self.robotname = robotname

    #Initialize blink support
    self.blinker = Blinker(robotname, to_dict(read_config(robotname + "_motors.yaml"), "name"))
    self.timer_blink = RandomTimer(self.blink, (5.0, 1.0))
    if robotname == "arthur":
      self.blinker.add_motor("Eyelid-Upper_L", 1.0)
      self.blinker.add_motor("Eyelid-Upper_R", 0.0)


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

  def blink_request(self, req):
    robotname, switch = req.data.split(":")
    if switch == "start":
      self.get_robot_ctrl(robotname).timer_blink.start()
    elif switch == "stop":
      self.get_robot_ctrl(robotname).timer_blink.stop()


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
    self.pub_pololu = {};
    self.pub_pololu['left'] = rospy.Publisher("left/command", MotorCommand, queue_size=30)
    self.pub_pololu['right'] = rospy.Publisher("right/command", MotorCommand, queue_size=30)
    rospy.Service("valid_coupled_face_exprs", ValidCoupledFaceExprs, self.valid_coupled_face_exprs)
    rospy.Subscriber("make_coupled_face_expr", MakeCoupledFaceExpr, self.coupled_face_request)
    rospy.Subscriber("cmd_blink", String, self.blink_request)

if __name__ == '__main__':
    HeadCtrl()
    rospy.loginfo("Started")
    rospy.spin()
