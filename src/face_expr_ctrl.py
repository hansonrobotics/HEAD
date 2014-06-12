#!/usr/bin/env python
import os
import yaml
from collections import OrderedDict
import rospy
from FaceExpr import FaceExpr
from ros_pololu_servo.msg import servo_pololu
from basic_head_api.srv import ValidFaceExprs
from basic_head_api.srv import MakeFaceExpr
from basic_head_api.msg import MotorPos

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
  dirname, filename = os.path.split(os.path.abspath(__file__))

  stream = open(os.path.join(dirname, configname), 'r')
  config = yaml.ordered_load(stream)
  stream.close()
  return config

def to_dict(list, key):
  result = {}
  for entry in list:
    result[entry[key]] = entry
  return result

class FaceExpressionCtrl:

  faces = None #Will hold a dictionary mapping expression names to FaceExpr instances.
  motorid_to_name = None #Will hold a dictionary mapping motorid numbers to motor names.
  pub = None #Will hold a cmd_pololu publisher

  def handle_valid_face_exprs(self, req):
    rospy.loginfo("Valid face expressions request.")
    return {'exprnames': self.faces.keys()}

  def handle_face_request(self, req):
    rospy.loginfo("Face request: {} of {}".format(req.intensity, req.str))

    pololu_cmds = self.faces[req.str].msgs(req.intensity)

    for cmd in pololu_cmds:
      self.pub.publish(cmd)

    # Return angles sent to ros_pololu_servo
    return {'motor_positions': map(
      lambda cmd: MotorPos(
        self.motorid_to_name[cmd.id],
        cmd.angle
      ),
      pololu_cmds
    )}

  def _fillFromConfig(self):
    motors_yaml = to_dict(read_config("einstein_motors.yaml"), "name")
    exprs_yaml = read_config("einstein.yaml")
    
    self.faces = FaceExpr.build_faces(
      exprs_yaml,
      motors_yaml
    )
    
    motorid_to_name = {}
    for motorname in motors_yaml:
      motorid_to_name[motors_yaml[motorname]['motorid']] = motorname
    self.motorid_to_name = motorid_to_name


  def init(self):
    self._fillFromConfig()

    rospy.init_node('face_expr_ctrl')
    rospy.Service('valid_face_exprs', ValidFaceExprs, self.handle_valid_face_exprs)
    rospy.Service("make_face_expr", MakeFaceExpr, self.handle_face_request)
    self.pub = rospy.Publisher('cmd_pololu', servo_pololu, queue_size=10)

if __name__ == '__main__':
    FaceExpressionCtrl().init()
    rospy.loginfo("Started")
    rospy.spin()