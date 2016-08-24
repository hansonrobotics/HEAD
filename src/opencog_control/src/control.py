#!/usr/bin/env python
import rospy

from dynamic_reconfigure.server import Server
from opencog_control.cfg import OpenpsiConfig

def callback(config, level):
    return config

if __name__ == "__main__":
    rospy.init_node("openpsi_control")
    srv = Server(OpenpsiConfig, callback)
    rospy.spin()
