#!/usr/bin/env python
import sys
import rospy

import roslib
import cv2

from pau2motors.msg import pau
from facial_puppetry.msg import land_marks
from cv_bridge import CvBridge, CvBridgeError
roslib.load_manifest('facial_puppetry')


'''
This Class performs Dlib to Blender Shapekey Mapping. The current capability of DLIB does not let us address
all shapekeys in Sophia.blend. For instance, mapping high resolution shapekeys like brow_inner_UP.L, brow_inner_UP.R ..
may not yield better result. Hence, those requirements are underlined as features to be added in this package.

 '''


class facial_puppetry:

  # declare publishers, subscribers, and other important static variables
  def __init__(self):
    self.pub_pau = rospy.Publisher('/blender_api/set_pau', pau, queue_size=10)
    self.baseline = []
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/dlib_values", land_marks, self.get_dlib_val)
    self.ref_width = 0
    self.ref_hight = 0
    self.cur_hight = 0
    self.cur_width = 0
    self.FLAG = 1
    self.count = 0
    self.shapekey_name = ['brow_center_UP', 'brow_center_DN', 'brow_inner_UP.L', 'brow_inner_DN.L', 'brow_inner_UP.R',
                          'brow_inner_DN.R', 'brow_outer_UP.L', 'brow_outer_DN.L', 'brow_outer_up.R', 'brow_outer_DN.R',
                          'eye-flare.UP.L', 'eye-blink.UP.L', 'eye-flare.UP.R', 'eye-blink.UP.R', 'eye-blink.LO.L',
                          'eye-flare.LO.L', 'eye-blink.LO.R', 'eye-flare.LO.R', 'wince.L', 'wince.R', 'sneer.L',
                          'sneer.R', 'eyes-look.dn', 'eyes-look.up', 'lip-UP.C.UP', 'lip-UP.C.DN', 'lip-UP.L.UP',
                          'lip-UP.L.DN', 'lip-UP.R.UP', 'lip-UP.R.DN', 'lips-smile.L','lips-smile.R', 'lips-wide.L',
                          'lips-narrow.L', 'lips-wide.R', 'lips-narrow.R', 'lip-DN.C.DN', 'lip-DN.C.UP', 'lip-DN.L.DN',
                          'lip-DN.L.UP', 'lip-DN.R.DN', 'lip-DN.R.UP', 'lips-frown.L', 'lips-frown.R','lip-JAW.DN']
    self.max = []
  

  # The final DLIB LM - Blender Shapekey mapping is here
  def map_to_Sophia(self, baseline_dist, current_dist, maxN, C_width, C_hight):
      final_val = []
      #put handled shapekey indexs here
      H_shapekeys = [0,2,4,6,8,10,11,12,13,14,15,16,17,20,21,30,31,32,34,44]

      if C_hight < 0:
          self.FLAG = -1
      else:
          self.FLAG = 1
      for i in range(0,45):
          MAX = 1; MIN = 0
          if i in H_shapekeys:
              if i == 44:
                  MAX = 0.5
              blend_val= min(MAX,max(MIN, ((current_dist[i] - baseline_dist[i]) + self.FLAG*(((current_dist[i] - baseline_dist[i]) * C_hight)/self.ref_hight))/maxN[i])) #make sure the values are in b/n 0 and 1
              final_val.append(blend_val)
          else: #if it is not handled put 0
              final_val.append(0.0)
      return final_val

  # callback: gets relative distance of DLIB's land marks per their corresponding shapekey
  def get_dlib_val(self, data):
      # sampling point: change this with event driven value
      if self.count == 5:
          self.baseline = data.dlib_val
          self.max = data.max_ref

          #not used for now
          self.ref_width = data.distW - data.distX
          self.ref_hight = data.distH - data.distY

      #change this with event_driven condition -- UI based
      if self.count >10:
          #Head Position and Orientation manipulation
          head_pau = pau()

          head_pau.m_headRotation.x = 0.9
          head_pau.m_headRotation.y = 0.5
          head_pau.m_headRotation.z = 0.7
          head_pau.m_headRotation.w = 0.9

          head_pau.m_headTranslation.x =0.9
          head_pau.m_headTranslation.y =0.7
          head_pau.m_headTranslation.z =0.8


          head_pau.m_neckRotation.x= -0.9
          head_pau.m_neckRotation.y= -0.5
          head_pau.m_neckRotation.z= 0.5
          head_pau.m_neckRotation.w= 0.9


          #Gaze Position and Orientation manipulation
          head_pau.m_eyeGazeLeftPitch = 0.01
          head_pau.m_eyeGazeLeftYaw = 0.1
          head_pau.m_eyeGazeRightPitch = 0.01
          head_pau.m_eyeGazeRightYaw = 0.1

          C_W = self.ref_width - self.cur_width
          C_H = self.ref_hight - self.cur_hight
          # assign final values to pau
          head_pau.m_coeffs = self.map_to_Sophia(self.baseline, data.dlib_val, self.max, C_W, C_H)
          head_pau.m_shapekeys = self.shapekey_name

          #publish to pau
          self.pub_pau.publish(head_pau)
      self.count += 1

def main(args):
    rospy.init_node('puppeteer_core', anonymous=True)
    facial_puppetry()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Facial Puppetry- Blendshape Mapper Shutting Down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
