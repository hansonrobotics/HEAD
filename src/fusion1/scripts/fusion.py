#!/usr/bin/env python2.7
import os
import rospy
import time

from fusion1.msg import CandidateUser,CandidateHand,CandidateSaliency,EstablishedUser,EstablishedHand,EstablishedSaliency


class Fusion(object):


    def __init__(self):

        self.rate = 30.0

        self.cuser_subs = []
        self.cuser_subs.append(rospy.Subscriber("lefteye/cuser",CandidateUser,self.HandleCandidateUser))
        self.cuser_subs.append(rospy.Subscriber("righteye/cuser",CandidateUser,self.HandleCandidateUser))
        self.cuser_subs.append(rospy.Subscriber("realsense/cuser",CandidateUser,self.HandleCandidateUser))
        self.cuser_subs.append(rospy.Subscriber("wideangle/cuser",CandidateUser,self.HandleCandidateUser))
        self.chand_subs = []
        self.chand_subs.append(rospy.Subscriber("lefteye/chand",CandidateHand,self.HandleCandidateHand))
        self.chand_subs.append(rospy.Subscriber("righteye/chand",CandidateHand,self.HandleCandidateHand))
        self.chand_subs.append(rospy.Subscriber("realsense/chand",CandidateHand,self.HandleCandidateHand))
        self.chand_subs.append(rospy.Subscriber("wideangle/chand",CandidateHand,self.HandleCandidateHand))
        self.csaliency_subs = []
        self.csaliency_subs.append(rospy.Subscriber("lefteye/csaliency",CandidateSaliency,self.HandleCandidateSaliency))
        self.csaliency_subs.append(rospy.Subscriber("righteye/csaliency",CandidateSaliency,self.HandleCandidateSaliency))
        self.csaliency_subs.append(rospy.Subscriber("realsense/csaliency",CandidateSaliency,self.HandleCandidateSaliency))
        self.csaliency_subs.append(rospy.Subscriber("wideangle/csaliency",CandidateSaliency,self.HandleCandidateSaliency))

        self.euser_pub = rospy.Publisher("user",EstablishedUser,queue_size=5)
        self.ehand_pub = rospy.Publisher("hand",EstablishedHand,queue_size=5)
        self.esaliency_pub = rospy.Publisher("saliency",EstablishedSaliency,queue_size=5)

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate),self.HandleTimer)


    def HandleCandidateUser(self,data):

        ()


    def HandleCandidateHand(self,data):

        ()


    def HandleCandidateSaliency(self,data):

        ()


    def HandleTimer(self,data):

        ts = data.current_expected

        # fuse candidate faces between pipelines

        # fuse candidate hands between pipelines

        # fuse candidate saliencies between pipelines

        # fuse faces and saliencies to improve saliency confidence

        # fuse hands and saliencies to improve saliency confidence

        # fuse sounds and faces

        # fuse sounds and hands

        # fuse sounds and saliency

        # fuse speech and faces

        # fuse speech and saliency

        # output established stuff


if __name__ == '__main__':


    rospy.init_node('fusion')
    node = Fusion()
    rospy.spin()
