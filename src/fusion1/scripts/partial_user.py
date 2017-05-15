#!/usr/bin/env python2.7
import os
import rospy
import numpy
import logging
import time
import cv2

from fusion1.msg import Face


# number of points needed for full confidence
full_points = 5


class PartialUser(object):


    def __init__(self):

        self.faces = [] # list of detected Faces
        self.age = 0.0 # estimated age
        self.age_confidence = 0.0 # confidence in age
        self.gender = 0 # estimated gender
        self.gender_confidence = 0.0 # confidence in gender
        self.identity = 0 # recognized user identity
        self.identity_confidence = 0.0 # confidence in identity


    def Extrapolate(self,ts):

        n = len(self.faces)
        if n < 2:
            print "not enough faces (%d), so outputting the only face available" % n
            return self.faces[0]

        # prepare for linear regression
        sumf = Face()
        sumff = Face()
        sumt = 0.0
        sumtt = 0.0
        sumft = Face()

        # iterate over last max_n faces only
        for face in self.faces:

            # time delta
            t = (ts - face.ts).to_sec()

            # face
            sumf.rect.origin.x += face.rect.origin.x
            sumf.rect.origin.y += face.rect.origin.y
            sumf.rect.size.x += face.rect.size.x
            sumf.rect.size.y += face.rect.size.y
            sumf.pos.x += face.pos.x
            sumf.pos.y += face.pos.y
            sumf.pos.z += face.pos.z
            sumf.confidence += face.confidence
            sumf.smile += face.smile
            sumf.frown += face.frown
            # expression strings cannot be extrapolated
            # landmarks might be useful here
            # thumb needs no extrapolation

            # face * face
            sumff.rect.origin.x += face.rect.origin.x * face.rect.origin.x
            sumff.rect.origin.y += face.rect.origin.y * face.rect.origin.y
            sumff.rect.size.x += face.rect.size.x * face.rect.size.x
            sumff.rect.size.y += face.rect.size.y * face.rect.size.y
            sumff.pos.x += face.pos.x * face.pos.x
            sumff.pos.y += face.pos.y * face.pos.y
            sumff.pos.z += face.pos.z * face.pos.z
            sumff.confidence += face.confidence * face.confidence
            sumff.smile += face.smile * face.smile
            sumff.frown += face.frown * face.frown

            # time
            sumt += t

            # time * time
            sumtt += t * t

            # face * time
            sumft.rect.origin.x += face.rect.origin.x * t
            sumft.rect.origin.y += face.rect.origin.y * t
            sumft.rect.size.x += face.rect.size.x * t
            sumft.rect.size.y += face.rect.size.y * t
            sumft.pos.x += face.pos.x * t
            sumft.pos.y += face.pos.y * t
            sumft.pos.z += face.pos.z * t
            sumft.confidence += face.confidence * t
            sumft.smile += face.smile * t
            sumft.frown += face.frown * t

        # face slope
        slpf = Face()
        den = n * sumtt - sumt * sumt
        slpf.rect.origin.x = (n * sumft.rect.origin.x - sumt * sumf.rect.origin.x) / den
        slpf.rect.origin.y = (n * sumft.rect.origin.y - sumt * sumf.rect.origin.y) / den
        slpf.rect.size.x = (n * sumft.rect.size.x - sumt * sumf.rect.size.x) / den
        slpf.rect.size.y = (n * sumft.rect.size.y - sumt * sumf.rect.size.y) / den
        slpf.pos.x = (n * sumft.pos.x - sumt * sumf.pos.x) / den
        slpf.pos.y = (n * sumft.pos.y - sumt * sumf.pos.y) / den
        slpf.pos.z = (n * sumft.pos.z - sumt * sumf.pos.z) / den
        slpf.confidence = (n * sumft.confidence - sumt * sumf.confidence) / den
        slpf.smile = (n * sumft.smile - sumt * sumf.smile) / den
        slpf.frown = (n * sumft.frown - sumt * sumf.frown) / den

        # result
        result = Face()
        result.ts = ts
        result.face_id = 0
        result.rect.origin.x = (sumf.rect.origin.x - slpf.rect.origin.x * sumt) / n
        result.rect.origin.y = (sumf.rect.origin.y - slpf.rect.origin.y * sumt) / n
        result.rect.size.x = (sumf.rect.size.x - slpf.rect.size.x * sumt) / n
        result.rect.size.y = (sumf.rect.size.y - slpf.rect.size.y * sumt) / n
        result.pos.x = (sumf.pos.x - slpf.pos.x * sumt) / n
        result.pos.y = (sumf.pos.y - slpf.pos.y * sumt) / n
        result.pos.z = (sumf.pos.z - slpf.pos.z * sumt) / n
        result.confidence = (sumf.confidence - slpf.confidence * sumt) / n
        result.smile = (sumf.smile - slpf.smile * sumt) / n
        result.frown = (sumf.frown - slpf.frown * sumt) / n

        return result


    def PruneBefore(self,ts):

        oldest = 0
        for i in range(0,len(self.faces)):
            if self.faces[i].ts.to_sec() < ts.to_sec():
                oldest = i + 1
        del self.faces[:oldest]


    def Append(self,face):

        self.faces.append(face)


    def CalculateConfidence(self):

        global full_faces

        # calculate confidence
        n = len(self.faces)
        if n > full_faces:
            n = full_faces
        total = 0.0
        for face in self.faces[-n:]:
            total += face.confidence
        total /= float(full_faces)

        return total
