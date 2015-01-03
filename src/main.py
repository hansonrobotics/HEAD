#! /usr/bin/env python

import rospy
from general_behavior import Tree

if __name__ == "__main__":
    rospy.init_node("Eva_Behavior")
    print("Starting Behavior Node")
    tree = Tree()
