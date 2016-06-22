#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import yaml
import rospkg

class image_converter:

  def __init__(self):
    rospy.init_node('image_converter', anonymous=True)
    self.filtered_face_locations = rospy.get_param('camera_topic')
    self.image_pub = rospy.Publisher(self.filtered_face_locations,Image)
    self.image_info = rospy.Publisher('camera_info',CameraInfo)
    self.bridge = CvBridge()
    cap = cv2.VideoCapture('/home/icog-labs/Videos/video.mp4')
    path = rospkg.RosPack().get_path("robots_config")
    path = path + "/robot/camera.yaml"
    stream = file(path, 'r')
    calib_data = yaml.load(stream)
    cam_info = CameraInfo()
    cam_info.width = calib_data['image_width']
    cam_info.height = calib_data['image_height']
    cam_info.K = calib_data['camera_matrix']['data']
    cam_info.D = calib_data['distortion_coefficients']['data']
    cam_info.R = calib_data['rectification_matrix']['data']
    cam_info.P = calib_data['projection_matrix']['data']
    cam_info.distortion_model = calib_data['distortion_model']
    while (not rospy.is_shutdown()) and (cap.isOpened()):
        self.keystroke = cv2.waitKey(1000 / 20)
        ret, frame = cap.read()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # cv2.imshow('frame', gray)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(gray, "mono8"))
        self.image_info.publish(cam_info)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)