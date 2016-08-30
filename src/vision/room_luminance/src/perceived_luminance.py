#!/usr/bin/env python
from sensor_msgs.msg import Image

from pi_face_tracker.msg import FaceEvent,Faces
from room_luminance.msg import Luminance
from cv_bridge import CvBridge, CvBridgeError
from collections import deque
import roslib
import rospy
import cv2
import sys
import  numpy as np
import math
import time


roslib.load_manifest('room_luminance')

'''
This Class contains the states and behaviours required to get the amount of light in the captured frame and detects object blocks above N% of coverage.
 '''

class ROIluminance:

 # initialize publishers, subscribers and static members inside class constructor.
  def __init__(self):
    self.pub = rospy.Publisher('/opencog/room_luminance', Luminance, queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.Visibility)
    self.face_event = rospy.Subscriber("/camera/face_locations", Faces, self.count_faces) # event = new_face: informs released/uncovered screen

    self.cntThresh = 100
    self.count = 25
    self.ref = ""
    self.RefArea = 0
    self.totalArea = 0
    self.covUp = 65
    self.covDown = 95
    self.Flag = 0
    self.face = 0
    self.waittime = 0



  '''
  # BGR/True Color based: could be used for other feature extraction. i.e hsv would be more than enough for the current req.

  def luminance_BGR(self, image_raw_bgr):
      #split into channels
      b, g, r = cv2.split(image_raw_bgr)
      size = np.size(image_raw_bgr)

      #Get average of Blue, Green and Red pixels
      B = float(np.sum(b)) / size
      G = float(np.sum(g)) / size
      R = float(np.sum(r)) / size

      # Photometric Luminance
      Y1 = 0.2126*R + 0.7152*G + 0.0722*B

      # Perceived Luminance
      Y2 = 0.299*R + 0.587*G + 0.114*B
      return [Y1, Y2]
  '''


  # HSV based Room Luminance Detection
  def luminance_HSV(self, image_raw_hsv):
      h, s, v = cv2.split(image_raw_hsv)
      size = np.size(image_raw_hsv)

      #  For other feature extraction purpose
      # H = float(np.sum(h)) / size #range 0-360
      # S = float(np.sum(s)) / size #range 0- 100

      V = float(np.sum(v)) / size #range 0- 100

      return float(V)


 # calculates the percent of screen covered by any object
  def objectBlock(self, image_raw):
    # image_raw is the absolute difference of the reference and the current frame
    # get the hight and width of a frame: important because of varity of camera hxw in different machines
    h, w = image_raw.shape
    self.totalArea = float(h * w)

    thresh = cv2.threshold(image_raw, 25, 255, cv2.THRESH_BINARY)[1]
    thresh = cv2.dilate(thresh, None, iterations=10)

    (cnts, _) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for c in cnts:
        # Discard objects/contours that usually does not have relevance if they are created by close enough/blocking objects
        if cv2.contourArea(c) <= self.cntThresh:
            pass
        self.RefArea = self.RefArea + cv2.contourArea(c)
    return math.ceil(((self.RefArea / self.totalArea) * 100)) #return the percent of covered space and round up the result


  # Regardless of the state of coverage this function returns  category of room light in general manner
  def classifyLuminance(self, lumene):
    if lumene <= 25:
        return "Dark"
    elif lumene <= 40:
        return "Nominal"
    else:
        return "Bright"
  def validate_cover(self):
      if self.count >= 5 and self.Flag == 1 and self.face >= 1:
          self.Flag = 0
          self.count = 25

      if self.Flag == 0:
          self.waittime = time.time()
      if time.time() - self.waittime >= 10:
          self.count = 25
          self.Flag = 0


  #checks for sudden changes from bright to dar or vice versa w/ 5 tolerance frames
  def sudden_change(self, room_light):
      dict = {"Dark": -10, "Nominal": 1, "Bright": 10}
      if len(d) < 5:
          d.append(dict[room_light])
      else:
          d.popleft()
          d.append(dict[room_light])

          if max(d) + min(d) == 0 and room_light =="Dark":
              return  -1
          if max(d) + min(d) == 0 and room_light =="Bright":
              return 1
          else:
              return 0


  # callback function for a topic "/camera/face_locations"
  def count_faces(self, face_state):
    self.face = len(face_state.faces)


  #callback function for a topic "/camera/image_raw"
  def Visibility(self, data):
    try:
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # default turn around frame: 25 - approximately 1s
        if self.count % 25 == 0:
            self.ref = gray
            self.count = 1
        self.count += 1

        #absolute difference of the ref. frame and the current frame - purpose:detect changes
        diff = cv2.absdiff(self.ref, gray)

        # get the luminance of HSV frame and put it to lumene var.
        lumene = self.luminance_HSV(hsv)

        # get the percent of coverage by an object and put it to coveraage var.
        coverage = self.objectBlock(diff)

        if coverage >= self.covUp:
            self.count = 1
            self.Flag = 1

        # cv2.imshow("Reference", self.ref)
        self.validate_cover()

        self.RefArea = 0
        room_light =self.classifyLuminance(lumene)



        msg = Luminance()
        msg.covered = self.Flag
        msg.room_light = room_light
        msg.sudden_change = self.sudden_change(room_light)

        msg.value = lumene
        msg.perc_covered = coverage

        self.pub.publish(msg)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            exit(0)

    except CvBridgeError as e:
      print(e)


def main(args):
    rospy.init_node('perceived_luminance', anonymous=True)
    ROIluminance()
    global d
    d = deque()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Luminance Detector Shutting Down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)