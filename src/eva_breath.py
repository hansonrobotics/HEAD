#!/usr/bin/python

import owyl
from owyl import blackboard
import rospy

class Tree():
    def __init__(self):
        self.blackboard = blackboard.Blackboard()
        self.tree = self.build_tree()
        while True:
            self.tree.next()

    def build_tree(self):
        eva_breath_tree = \
            owyl.repeatAlways(
                owyl.sequence(
                    # TODO: Behaviors of breath (TBC)
                    self.breathe()
                )
            )
        return owyl.visit(eva_breath_tree, blackboard=self.blackboard)

    @owyl.taskmethod
    def breathe(self, **kwargs):
        # TODO
        yield True

if __name__ == "__main__":
    rospy.init_node("Eva_Breath")
    tree = Tree()
