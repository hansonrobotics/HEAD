#!/usr/bin/env python
import os
import yaml
import rospy
from EinsteinFace import EinsteinFace
from ros_pololu_servo.msg import servo_pololu
from basic_head_api.msg import FaceExprReq

def read_config(configname):
  dirname, filename = os.path.split(os.path.abspath(__file__))

  stream = open(os.path.join(dirname, configname), 'r')
  config = yaml.load(stream)
  return config

def to_dict(list, key):
  result = {}
  for entry in list:
    result[entry[key]] = entry
  return result

class FaceExpressionCtrl:

  def on_request(self, req):
    rospy.loginfo("Face request: {} of {}".format(req.intensity, req.str))

    pololu_cmds = self.faces[req.str].msgs(req.intensity)

    for cmd in pololu_cmds:
      self.pub.publish(cmd)

  def init(self):
    #Will hold a dictionary mapping expression names to EinsteinFace instances.
    self.faces = EinsteinFace.build_faces(
      read_config("einstein.yaml"),
      to_dict(read_config("einstein_motors.yaml"), "name")
    )
    
    rospy.init_node('face_expr_ctrl')
    self.pub = rospy.Publisher('/cmd_pololu', servo_pololu, queue_size=10)
    rospy.Subscriber("/cmd_face_expr", FaceExprReq, self.on_request)

if __name__ == '__main__':
    FaceExpressionCtrl().init()
    rospy.loginfo("Started")
    rospy.spin()