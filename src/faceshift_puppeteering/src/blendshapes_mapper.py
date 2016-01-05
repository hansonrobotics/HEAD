#!/usr/bin/env python
__author__ = 'tesfa'

import rospy

from dynamic_reconfigure.server import Server
from faceshift_puppeteering.cfg import FBConfig

def callback(config, level):
    # Handle the call back here.
    return config

if __name__ == "__main__":
    rospy.init_node("faceshift_puppeteering_mapper", anonymous = True)

    srv = Server(FBConfig, callback)
    rospy.spin()


