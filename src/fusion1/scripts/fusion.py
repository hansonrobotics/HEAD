#!/usr/bin/env python2.7
import os
import rospy

from vision_pipeline import VisionPipeline
from hearing_pipeline import HearingPipeline


class Fusion(object):


    def __init__(self):

        #self.lefteye = VisionPipeline("lefteye")
        #self.righteye = VisionPipeline("righteye")
        #self.realsense = VisionPipeline("realsense")
        self.wideangle = VisionPipeline("wideangle")

        self.period_sec = 0.03333 # 30Hz
        self.timer = rospy.Timer(rospy.Duration(self.period_sec),self.HandleTimer)


    def HandleTimer(self,data):

        ts = data.current_expected

        #self.lefteye.Step(ts)
        #self.righteye.Step(ts)
        #self.realsense.Step(ts)
        self.wideangle.Step(ts)


if __name__ == '__main__':

    rospy.init_node('fusion')
    node = Fusion()
    rospy.spin()
