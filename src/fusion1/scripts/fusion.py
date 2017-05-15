#!/usr/bin/env python2.7
import os
import rospy
import time

from vision_pipeline import VisionPipeline
from hearing_pipeline import HearingPipeline


invocation_id = hash(time.strftime("%H%M%S")) & 0xFFFFFFFF


class Fusion(object):


    def __init__(self):

        global invocation_id

        #self.lefteye = VisionPipeline("lefteye",invocation_id)
        #self.righteye = VisionPipeline("righteye",invocation_id)
        #self.realsense = VisionPipeline("realsense",invocation_id)
        self.wideangle = VisionPipeline("wideangle",invocation_id)

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
