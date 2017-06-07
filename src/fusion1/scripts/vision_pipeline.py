#!/usr/bin/env python2.7

# R2 Perception - Hanson Robotics Unified Perception System, v1.0
# by Desmond Germans

import os
import rospy
import numpy
import time
import cv2

from dynamic_reconfigure.server import Server
from fusion1.cfg import VisionConfig
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from math import sqrt
from fusion1.msg import Face,Hand,Saliency,FaceRequest,FaceResponse,CandidateUser,CandidateHand,CandidateSaliency
from visualization_msgs.msg import Marker

opencv_bridge = CvBridge()


thumb_base_dir = os.getenv("HOME") + "/hansonrobotics/HEAD/src/fusion1/test/thumbs"
thumb_ext = ".png"

face_continuity_threshold_m = 0.5

hand_continuity_threshold_m = 0.5

minimum_confidence = 0.4

time_difference = rospy.Time(1,0)

full_points = 5


serial_number = 0

def GenerateCandidateUserID():
    global serial_number
    result = serial_number
    serial_number += 1
    return result

def GenerateCandidateHandID():
    global serial_number
    result = serial_number
    serial_number += 1
    return result

def GenerateCandidateSaliencyID():
    global serial_number
    result = serial_number
    serial_number += 1
    return result


class CandidateUserPredictor(object):


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
            return self.faces[0]

        # prepare for linear regression
        sumf = Face()
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
            sumf.position.x += face.position.x
            sumf.position.y += face.position.y
            sumf.position.z += face.position.z
            sumf.confidence += face.confidence
            sumf.smile += face.smile
            sumf.frown += face.frown
            # expression strings cannot be extrapolated
            # landmarks might be useful here
            # thumb needs no extrapolation

            # time
            sumt += t

            # time * time
            sumtt += t * t

            # face * time
            sumft.rect.origin.x += face.rect.origin.x * t
            sumft.rect.origin.y += face.rect.origin.y * t
            sumft.rect.size.x += face.rect.size.x * t
            sumft.rect.size.y += face.rect.size.y * t
            sumft.position.x += face.position.x * t
            sumft.position.y += face.position.y * t
            sumft.position.z += face.position.z * t
            sumft.confidence += face.confidence * t
            sumft.smile += face.smile * t
            sumft.frown += face.frown * t

        # face slope
        slpf = Face()
        den = n * sumtt - sumt * sumt
        if den == 0.0:
            print "division by zero when calculating regression slope (n = %d), this shouldn't happen" % n
            return self.faces[0]

        slpf.rect.origin.x = (n * sumft.rect.origin.x - sumt * sumf.rect.origin.x) / den
        slpf.rect.origin.y = (n * sumft.rect.origin.y - sumt * sumf.rect.origin.y) / den
        slpf.rect.size.x = (n * sumft.rect.size.x - sumt * sumf.rect.size.x) / den
        slpf.rect.size.y = (n * sumft.rect.size.y - sumt * sumf.rect.size.y) / den
        slpf.position.x = (n * sumft.position.x - sumt * sumf.position.x) / den
        slpf.position.y = (n * sumft.position.y - sumt * sumf.position.y) / den
        slpf.position.z = (n * sumft.position.z - sumt * sumf.position.z) / den
        slpf.confidence = (n * sumft.confidence - sumt * sumf.confidence) / den
        slpf.smile = (n * sumft.smile - sumt * sumf.smile) / den
        slpf.frown = (n * sumft.frown - sumt * sumf.frown) / den

        # result
        if n == 0.0:
            print "divsion by zero when calculating regression result (n = %d), this shouldn't happen" % n
            return self.faces[0]

        result = Face()
        result.ts = ts
        result.face_id = 0
        result.rect.origin.x = (sumf.rect.origin.x - slpf.rect.origin.x * sumt) / n
        result.rect.origin.y = (sumf.rect.origin.y - slpf.rect.origin.y * sumt) / n
        result.rect.size.x = (sumf.rect.size.x - slpf.rect.size.x * sumt) / n
        result.rect.size.y = (sumf.rect.size.y - slpf.rect.size.y * sumt) / n
        result.position.x = (sumf.position.x - slpf.position.x * sumt) / n
        result.position.y = (sumf.position.y - slpf.position.y * sumt) / n
        result.position.z = (sumf.position.z - slpf.position.z * sumt) / n
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

        global full_points

        # calculate confidence
        n = len(self.faces)
        if n > full_points:
            n = full_points
        total = 0.0
        for face in self.faces[-n:]:
            total += face.confidence
        total /= float(full_points)

        return total


class CandidateHandPredictor(object):


    def __init__(self):

        self.hands = [] # list of detected Hands


    def Extrapolate(self,ts):

        n = len(self.hands)
        if n < 2:
            return self.hands[0]

        # prepare for linear regression
        sumh = Hand()
        sumt = 0.0
        sumtt = 0.0
        sumht = Hand()

        # iterate over last max_n hands only
        for hand in self.hands:

            # time delta
            t = (ts - hand.ts).to_sec()

            # hand
            sumh.position.x += hand.position.x
            sumh.position.y += hand.position.y
            sumh.position.z += hand.position.z
            sumh.confidence += hand.confidence

            # time
            sumt += t

            # time * time
            sumtt += t * t

            # hand * time
            sumht.position.x += hand.position.x * t
            sumht.position.y += hand.position.y * t
            sumht.position.z += hand.position.z * t
            sumht.confidence += hand.confidence * t

        # hand slope
        slph = Hand()
        den = n * sumtt - sumt * sumt
        if den == 0.0:
            print "division by zero when calculating regression slope (n = %d), this shouldn't happen" % n
            return self.hands[0]

        slph.position.x = (n * sumht.position.x - sumt * sumh.position.x) / den
        slph.position.y = (n * sumht.position.y - sumt * sumh.position.y) / den
        slph.position.z = (n * sumht.position.z - sumt * sumh.position.z) / den
        slph.confidence = (n * sumht.confidence - sumt * sumh.confidence) / den

        # result
        if n == 0.0:
            print "divsion by zero when calculating regression result (n = %d), this shouldn't happen" % n
            return self.faces[0]

        result = Hand()
        result.ts = ts
        result.hand_id = 0
        result.position.x = (sumh.position.x - slph.position.x * sumt) / n
        result.position.y = (sumh.position.y - slph.position.y * sumt) / n
        result.position.z = (sumh.position.z - slph.position.z * sumt) / n
        result.confidence = (sumh.confidence - slph.confidence * sumt) / n

        return result


    def PruneBefore(self,ts):

        oldest = 0
        for i in range(0,len(self.hands)):
            if self.hands[i].ts.to_sec() < ts.to_sec():
                oldest = i + 1
        del self.hands[:oldest]


    def Append(self,hand):

        self.hands.append(hand)


    def CalculateConfidence(self):

        global full_points

        # calculate confidence
        n = len(self.hands)
        if n > full_points:
            n = full_points
        total = 0.0
        for hand in self.hands[-n:]:
            total += hand.confidence
        total /= float(full_points)

        return total


class CandidateSaliencyPredictor(object):


    def __init__(self):

        self.hands = [] # list of detected Hands


    def Extrapolate(self,ts):

        n = len(self.saliencies)
        if n < 2:
            return self.saliencies[0]

        # prepare for linear regression
        sums = Saliency()
        sumt = 0.0
        sumtt = 0.0
        sumst = Saliency()

        # iterate over last max_n saliencies only
        for saliency in self.saliencies:

            # time delta
            t = (ts - saliency.ts).to_sec()

            # saliency
            sums.position.x += saliency.position.x
            sums.position.y += saliency.position.y
            sums.position.z += saliency.position.z
            sums.confidence += saliency.confidence

            # time
            sumt += t

            # time * time
            sumtt += t * t

            # saliency * time
            sumst.position.x += saliency.position.x * t
            sumst.position.y += saliency.position.y * t
            sumst.position.z += saliency.position.z * t
            sumst.confidence += saliency.confidence * t

        # saliency slope
        slph = Saliency()
        den = n * sumtt - sumt * sumt
        if den == 0.0:
            print "division by zero when calculating regression slope (n = %d), this shouldn't happen" % n
            return self.saliencies[0]

        slps.position.x = (n * sumst.position.x - sumt * sums.position.x) / den
        slps.position.y = (n * sumst.position.y - sumt * sums.position.y) / den
        slps.position.z = (n * sumst.position.z - sumt * sums.position.z) / den
        slps.confidence = (n * sumst.confidence - sumt * sums.confidence) / den

        # result
        if n == 0.0:
            print "divsion by zero when calculating regression result (n = %d), this shouldn't happen" % n
            return self.saliencies[0]

        result = Hand()
        result.ts = ts
        result.hand_id = 0
        result.position.x = (sumh.position.x - slph.position.x * sumt) / n
        result.position.y = (sumh.position.y - slph.position.y * sumt) / n
        result.position.z = (sumh.position.z - slph.position.z * sumt) / n
        result.confidence = (sumh.confidence - slph.confidence * sumt) / n

        return result


    def PruneBefore(self,ts):

        oldest = 0
        for i in range(0,len(self.saliencies)):
            if self.saliencies[i].ts.to_sec() < ts.to_sec():
                oldest = i + 1
        del self.saliencies[:oldest]


    def Append(self,hand):

        self.saliencies.append(saliency)


    def CalculateConfidence(self):

        global full_points

        # calculate confidence
        n = len(self.saliencies)
        if n > full_points:
            n = full_points
        total = 0.0
        for saliency in self.saliencies[-n:]:
            total += saliency.confidence
        total /= float(full_points)

        return total


class VisionPipeline(object):


    def __init__(self):

        self.debug  =rospy.get_param("/debug")
        self.store_thumbs = rospy.get_param("/store_thumbs")
        self.visualization = rospy.get_param("/visualization")

        self.name = os.path.basename(rospy.get_namespace())
        self.session_tag = rospy.get_param("/session_tag")
        self.session_id = hash(self.session_tag) & 0xFFFFFFFF

        self.camera_id = hash(self.name) & 0xFFFFFFFF

        if self.store_thumbs:
            today_tag = time.strftime("%Y%m%d")
            camera_tag = self.name + "_%08X" % (self.camera_id & 0xFFFFFFFF)
            self.thumb_dir = thumb_base_dir + "/" + today_tag + "/" + self.session_tag + "_%08X/" % (self.session_id & 0xFFFFFFFF) + camera_tag + "/"
            if not os.path.exists(self.thumb_dir):
                os.makedirs(self.thumb_dir)

        self.cusers = {}
        self.chands = {}
        self.csaliencies = {}

        # subscribers for raw face, hand and saliency nodes
        self.face_sub = rospy.Subscriber("raw_face",Face,self.HandleFace)
        self.hand_sub = rospy.Subscriber("raw_hand",Hand,self.HandleHand)
        self.saliency_sub = rospy.Subscriber("raw_saliency",Saliency,self.HandleSaliency)

        # request/response to/from face analysis node
        self.face_response_sub = rospy.Subscriber("face_response",FaceResponse,self.HandleFaceResponse)
        self.face_request_pub = rospy.Publisher("face_request",FaceRequest,queue_size=5)

        # publishers for candidate user, hand and saliency
        self.cuser_pub = rospy.Publisher("cuser",CandidateUser,queue_size=5)
        self.chand_pub = rospy.Publisher("chand",CandidateHand,queue_size=5)
        self.csaliency_pub = rospy.Publisher("csaliency",CandidateSaliency,queue_size=5)

        # rviz markers
        if self.visualization:
            self.face_rviz_pub = rospy.Publisher("rviz_face",Marker,queue_size=5)
            self.hand_rviz_pub = rospy.Publisher("rviz_hand",Marker,queue_size=5)
            self.saliency_rviz_pub = rospy.Publisher("rviz_saliency",Marker,queue_size=5)

        self.config_srv = Server(VisionConfig,self.HandleConfig)

        # for debugging
        if self.debug:
            cv2.startWindowThread()
            cv2.namedWindow(self.name)
            self.frame_sub = rospy.Subscriber(self.name + "/camera/image_raw",Image,self.HandleFrame)


    def HandleConfig(self,data,level):

        print "{}".format(data)
        return data


    def HandleFace(self,data):

        global face_continuity_threshold_m

        if data.ts.secs == 0:
            data.ts = rospy.get_rostime()

        closest_cuser_id = 0
        closest_dist = face_continuity_threshold_m

        for cuser_id in self.cusers:
            face = self.cusers[cuser_id].Extrapolate(data.ts)
            dx = data.position.x - face.position.x
            dy = data.position.y - face.position.y
            dz = data.position.z - face.position.z
            d = sqrt(dx * dx + dy * dy + dz * dz)
            if (closest_cuser_id == 0) or (d < closest_dist):
                closest_cuser_id = cuser_id
                closest_dist = d

        if closest_dist < face_continuity_threshold_m:

            self.cusers[closest_cuser_id].Append(data)

        else:

            closest_cuser_id = GenerateCandidateUserID()
            cuser = CandidateUserPredictor()
            cuser.Append(data)

            self.cusers[closest_cuser_id] = cuser

            # send face analysis request to face_analysis
            msg = FaceRequest()
            msg.session_id = self.session_id
            msg.camera_id = self.camera_id
            msg.cuser_id = closest_cuser_id
            msg.face_id = data.face_id
            msg.ts = data.ts
            msg.thumb = data.thumb
            self.face_request_pub.publish(msg)

        if self.store_thumbs:
            cuser_tag = "cuser_%08X" % (closest_cuser_id & 0xFFFFFFFF)
            dir = self.thumb_dir + cuser_tag + "/"
            if not os.path.exists(dir):
                os.makedirs(dir)
            thumb = opencv_bridge.imgmsg_to_cv2(data.thumb)
            face_tag = "face_%08X" % (data.face_id & 0xFFFFFFFF)
            thumb_file = dir + face_tag + thumb_ext
            cv2.imwrite(thumb_file,thumb)


    def HandleHand(self,data):

        global hand_continuity_threshold_m

        if data.ts.secs == 0:
            data.ts = rospy.get_rostime()

        closest_chand_id = 0
        closest_dist = hand_continuity_threshold_m

        for chand_id in self.chands:
            hand = self.chands[chand_id].Extrapolate(data.ts)
            dx = data.position.x - hand.position.x
            dy = data.position.y - hand.position.y
            dz = data.position.z - hand.position.z
            d = sqrt(dx * dx + dy * dy + dz * dz)
            if (closest_chand_id == 0) or (d < closest_dist):
                closest_chand_id = chand_id
                closest_dist = d

        if closest_dist < hand_continuity_threshold_m:

            self.chands[closest_chand_id].Append(data)

        else:

            closest_chand_id = GenerateCandidateHandID()
            chand = CandidateHandPredictor()
            chand.Append(data)

            self.chands[closest_chand_id] = chand


    def HandleSaliency(self,data):

        global saliency_continuity_threshold_m

        if data.ts.secs == 0:
            data.ts = rospy.get_rostime()

        closest_csaliency_id = 0
        closest_dist = saliency_continuity_threshold_m

        for csaliency_id in self.csaliencies:
            saliency = self.csaliencies[csaliency_id].Extrapolate(data.ts)
            dx = data.position.x - hand.position.x
            dy = data.position.y - hand.position.y
            dz = data.position.z - hand.position.z
            d = sqrt(dx * dx + dy * dy + dz * dz)
            if (closest_chand_id == 0) or (d < closest_dist):
                closest_chand_id = chand_id
                closest_dist = d

        if closest_dist < saliency_continuity_threshold_m:

            self.csaliencies[closest_csaliency_id].Append(data)

        else:

            closest_csaliency_id = GenerateCandidateSaliencyID()
            csaliency = CandidateSaliencyPredictor()
            csaliency.Append(data)

            self.csaliencies[closest_csaliency_id] = csaliency


    def HandleFaceResponse(self,data):

        if data.cuser_id not in self.cusers:
            return

        self.cusers[data.cuser_id].age = data.age
        self.cusers[data.cuser_id].age_confidence = data.age_confidence
        self.cusers[data.cuser_id].gender = data.gender
        self.cusers[data.cuser_id].gender_confidence = data.gender_confidence
        self.cusers[data.cuser_id].identity = data.identity
        self.cusers[data.cuser_id].identity_confidence = data.identity_confidence


    # for debugging
    def HandleFrame(self,data):

        # get image
        cvimage = opencv_bridge.imgmsg_to_cv2(data,"bgr8")

        # get time
        ts = rospy.get_rostime()

        # display users
        for cuser_id in self.cusers:

            # the face
            face = self.cusers[cuser_id].Extrapolate(ts)
            x = int((0.5 + 0.5 * face.position.x) * 640.0)
            y = int((0.5 + 0.5 * face.position.y) * 480.0)
            cv2.circle(cvimage,(x,y),10,(0,255,255),2)

            # annotate with info if available
            label = ""
            if self.cusers[cuser_id].identity_confidence > 0.0:
                label = self.cusers[cuser_id].identity
            if self.cusers[cuser_id].age_confidence > 0.0:
                if label != "":
                    label += ", "
                label += str(int(self.cusers[cuser_id].age)) + " y/o"
            if self.cusers[cuser_id].gender_confidence > 0.0:
                if label != "":
                    label += ", "
                if gender == 0:
                    label += "genderless"
                elif gender == 1:
                    label += "female"
                elif gender == 2:
                    label += "male"
            cv2.putText(cvimage,label,(x - 20,y + 20),cv2.cv.CV_FONT_HERSHEY_PLAIN,1,(0,255,255))

        cv2.imshow(self.name,cvimage)


    def HandleTimer(self,data):

        global minimum_confidence
        global time_difference

        ts = data.current_expected

        # mine the current candidate users for confident ones
        for cuser_id in self.cusers:
            conf = self.cusers[cuser_id].CalculateConfidence()
            if conf >= minimum_confidence:
                msg = CandidateUser()
                msg.session_id = self.session_id
                msg.camera_id = self.camera_id
                msg.cuser_id = cuser_id
                msg.ts = ts
                cuser = self.cusers[cuser_id].Extrapolate(ts)
                msg.position = cuser.position
                msg.confidence = cuser.confidence
                msg.smile = cuser.smile
                msg.frown = cuser.frown
                msg.expressions = cuser.expressions
                msg.landmarks = cuser.landmarks
                msg.age = cuser.age
                msg.age_confidence = cuser.age_confidence
                msg.gender = cuser.gender
                msg.gender_confidence = cuser.gender_confidence
                msg.identity = cuser.identity
                msg.identity_confidence = cuser.identity_confidence
                self.cuser_pub.publish(msg)

                # TODO: output markers to rviz

        # prune the candidate users and remove them if they disappeared
        to_be_removed = []
        prune_before_time = ts - time_difference
        for cuser_id in self.cusers:
            self.cusers[cuser_id].PruneBefore(prune_before_time)
            if len(self.cusers[cuser_id].faces) == 0:
                to_be_removed.append(cuser_id)
        for key in to_be_removed:
            del self.cusers[cuser_id]


        # mine the current candidate hands for confident ones
        for chand_id in self.chands:
            conf = self.chands[chand_id].CalculateConfidence()
            if conf >= minimum_confidence:
                msg = CandidateHand()
                msg.session_id = self.session_id
                msg.camera_id = self.camera_id
                msg.chand_id = chand_id
                msg.ts = ts
                chand = self.chands[chand_id].Extrapolate(ts)
                msg.position = chand.position
                msg.confidence = chand.confidence
                self.chand_pub.publish(msg)

                # TODO: output markers to rviz

        # prune the candidate hands and remove them if they disappeared
        to_be_removed = []
        prune_before_time = ts - time_difference
        for chand_id in self.phands:
            self.chands[chand_id].PruneBefore(prune_before_time)
            if len(self.chands[chand_id].hands) == 0:
                to_be_removed.append(chand_id)
        for key in to_be_removed:
            del self.chands[chand_id]


        # mine the current candidate saliencies for confident ones
        for csaliency_id in self.csaliencies:
            conf = self.csaliencies[csaliency_id].CalculateConfidence()
            if conf >= minimum_confidence:
                msg = CandidateSaliency()
                msg.session_id = self.session_id
                msg.camera_id = self.camera_id
                msg.csaliency_id = csaliency_id
                msg.ts = ts
                csaliency = self.csaliencies[csaliency_id].Extrapolate(ts)
                msg.position = csaliency.position
                msg.confidence = csaliency.confidence
                self.csaliency_pub.publish(msg)

                # TODO: output markers to rviz

        # prune the candidate saliencies and remove them if they disappeared
        to_be_removed = []
        prune_before_time = ts - time_difference
        for csaliency_id in self.csaliencies:
            self.csaliencies[csaliency_id].PruneBefore(prune_before_time)
            if len(self.psaliencies[csaliency_id].hands) == 0:
                to_be_removed.append(csaliency_id)
        for key in to_be_removed:
            del self.csaliencies[csaliency_id]


        # output markers to rviz
        #for puser_id in self.pusers:
        #    conf = self.pusers[puser_id].CalculateConfidence()
        #    if conf >= minimum_confidence:
        #        face = self.pusers[puser_id].Extrapolate(ts)
        #        marker = Marker()
        #        marker.header.frame_id = "world"
        #        marker.header.stamp = ts
        #        marker.id = puser_id
        #        marker.ns = "/robot/perception/" + self.name
        #        marker.type = Marker.SPHERE
        #        marker.action = Marker.MODIFY
        #        marker.pose.position.x = face.pos.x
        #        marker.pose.position.y = face.pos.y
        #        marker.pose.position.z = face.pos.z
        #        marker.pose.orientation.x = 0.0
        #        marker.pose.orientation.y = 0.0
        #        marker.pose.orientation.z = 0.0
        #        marker.pose.orientation.w = 1.0
        #        marker.scale.x = 0.1
        #        marker.scale.y = 0.1
        #        marker.scale.z = 0.1
        #        marker.color.r = 1.0
        #        marker.color.g = 0.0
        #        marker.color.b = 0.0
        #        marker.color.a = 1.0
        #        marker.lifetime = rospy.Duration(2,0)
        #        marker.frame_locked = False
        #        self.face_rviz_pub.publish(marker)


if __name__ == '__main__':

    rospy.init_node('vision_pipeline')
    node = VisionPipeline()
    rospy.spin()
