#!/usr/bin/python
import owyl
import rospy
import roslib
from owyl import blackboard


class Tree():
    def __init__(self):
        self.blackboard = blackboard.Blackboard()
        self.tree = self.build_tree()
        while True:
            self.tree.next()

    def build_tree(self):
        eva_blink_tree = \
            owyl.repeatAlways(
                owyl.sequence(
                    # TODO: Behaviors of blink (TBC)
                    self.blink()
                )
            )
        return owyl.visit(eva_blink_tree, blackboard=self.blackboard)

    @owyl.taskmethod
    def blink(self, **kwargs):
        # TODO
        yield True

if __name__ == "__main__":
    rospy.init_node("Eva_Blink")
    tree = Tree()