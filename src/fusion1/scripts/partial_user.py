#!/usr/bin/env python2.7
import os
import rospy
import numpy
import logging
import time
import cv2

# number of points for 100% confidence
full_points = 5


class Point(object):


    def __init__(self,ts=0,x=0.0,y=0.0,z=0.0):

        self.ts = ts # timestamp
        self.x = x
        self.y = y
        self.z = z


class PartialUser(object):


    def __init__(self):

        self.points = [] # list of Points
        self.age = 0
        self.age_confidence = 0.0
        self.female = False
        self.female_confidence = 0.0


    def Extrapolate(self,ts):

        n = len(self.points)
        if n < 2:
            return self.points[0]
        sumx = 0.0
        sumxx = 0.0
        sumy = 0.0
        sumyy = 0.0
        sumz = 0.0
        sumzz = 0.0
        sumt = 0.0
        sumtt = 0.0
        sumtx = 0.0
        sumty = 0.0
        sumtz = 0.0
        for point in self.points:
            x = point.x
            y = point.y
            z = point.z
            t = (point.ts - ts).to_sec()
            sumx += x
            sumxx += x * x
            sumy += y
            sumyy += y * y
            sumz += z
            sumzz += z * z
            sumt += t
            sumtt += t * t
            sumtx += t * x
            sumty += t * y
            sumtz += t * z
        xslp = (n * sumtx - sumt * sumx) / (n * sumtt - sumt * sumt)
        rx = (sumx - xslp * sumt) / n
        yslp = (n * sumty - sumt * sumy) / (n * sumtt - sumt * sumt)
        ry = (sumy - yslp * sumt) / n
        zslp = (n * sumtz - sumt * sumz) / (n * sumtt - sumt * sumt)
        rz = (sumz - zslp * sumt) / n
        return Point(ts,rx,ry,rz)


    def PruneBefore(self,ts):

        oldest = 0
        for i in range(0,len(self.points)):
            if self.points[i].ts.to_sec() < ts.to_sec():
                oldest = i + 1 # because Python...
        del self.points[:oldest]


    def Append(self,point):

        self.points.append(point)


    def CalculateConfidence(self):

        global full_points

        n = len(self.points)
        if n > full_points:
            n = full_points
        return float(n) / float(full_points)
