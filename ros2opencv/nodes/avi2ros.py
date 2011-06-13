#!/usr/bin/env python

""" avi2ros.py - Version 0.1 2011-04-28

    Read in an AVI video file and republish as a ROS Image topic.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2011 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import roslib
roslib.load_manifest('ros2opencv')
import sys
import rospy
import cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class AVI2ROS:
    def __init__(self):
        rospy.init_node('avi2ros', anonymous=True)
        
        self.input = rospy.get_param("~input", "")
        image_pub = rospy.Publisher("output", Image)
        
        rospy.on_shutdown(self.cleanup)
        
        video = cv.CaptureFromFile(self.input)
        fps = int(cv.GetCaptureProperty(video, cv.CV_CAP_PROP_FPS))
        
        """ Bring the fps up to 25 Hz """
        fps = int(fps * 25.0 / fps)
    
        cv.NamedWindow("Image window", True) # autosize the display

        bridge = CvBridge()
                
        self.paused = False
        self.keystroke = None
        self.restart = False
    
        while not rospy.is_shutdown():
            
            if self.restart:
                video = cv.CaptureFromFile(self.input)
                self.restart = None
            
            """ handle events """
            self.keystroke = cv.WaitKey(1000 / fps)
            
            """ Process any keyboard commands """
            if 32 <= self.keystroke and self.keystroke < 128:
                cc = chr(self.keystroke).lower()
                if cc == 'q':
                    """ user has press the q key, so exit """
                    rospy.signal_shutdown("User hit q key to quit.")
                elif cc == ' ':
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
    
    def cleanup(self):
            print "Shutting down vision node."
            cv.DestroyAllWindows()

def main(args):
    help_message =  "Hot keys: \n" \
          "\tq     - quit the program\n" \
          "\tr     - restart video from beginning\n" \
          "\tspace - toggle pause/play\n"

    print help_message
    
    try:
        a2r = AVI2ROS()
    except KeyboardInterrupt:
        print "Shutting down avi2ros..."
        cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
