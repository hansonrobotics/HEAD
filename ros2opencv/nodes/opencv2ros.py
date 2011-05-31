#!/usr/bin/env python
import roslib
roslib.load_manifest('pi_opencv')
import sys
import rospy
import cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class OpenCV2ROS:
    
    def cleanup(self):
            print "Shutting down vision node."
            cv.DestroyAllWindows()  

    def __init__(self, path):
        rospy.init_node('opencv2ros', anonymous=True)
        
        image_pub = rospy.Publisher("/camera/rgb/image_color", Image)
        
        rospy.on_shutdown(self.cleanup)
    
        #video = cv.CaptureFromCAM(0)
        video = cv.CaptureFromFile(path)
        fps = int(cv.GetCaptureProperty(video, cv.CV_CAP_PROP_FPS) * 1.4)
        #rospy.loginfo(cv.GetCaptureProperty(video, cv.CV_CAP_PROP_FRAME_WIDTH))
    
        cv.NamedWindow("Image window", cv.CV_NORMAL)
        cv.ResizeWindow("Image window", 320, 240)

        bridge = CvBridge()
                
        self.paused = False
        self.keystroke = None
        self.restart = False
    
        while not rospy.is_shutdown():
            
            if self.restart:
                video = cv.CaptureFromFile(path)
                self.restart = None
            
            """ handle events """
            self.keystroke = cv.WaitKey(1000 / fps)
            
            """ Process any keyboard commands """
            if 32 <= self.keystroke and self.keystroke < 128:
                cc = chr(self.keystroke).lower()
                if cc == 'q':
                    """ user has press the q key, so exit """
                    rospy.signal_shutdown("User hit q key to quit.")
                elif cc == 'p' or cc == ' ':
                    """ Pause or continue the video """
                    self.paused = not self.paused
                elif cc == 'r':
                    self.restart = True

            if self.paused:
                rospy.sleep(1)
                continue
    
            frame = cv.QueryFrame(video)
            
            cv.ShowImage("Image window", frame)
            
            try:
                image_pub.publish(bridge.cv_to_imgmsg(frame, "bgr8"))
            except CvBridgeError, e:
                print e
  

def main(args):
    try:
        o2r = OpenCV2ROS(sys.argv[1])
    except KeyboardInterrupt:
        print "Shutting down opencv2ros..."
        cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
