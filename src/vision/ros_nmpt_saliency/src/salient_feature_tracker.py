#!/usr/bin/env python

import rospy

from blender_api_msgs.msg import Target
from ros_nmpt_saliency.msg import targets
from pi_face_tracker.msg import FaceEvent, Faces

'''
    This is a sample subscriber written to control Sophia's Head movement based on the degree of Salient points.
'''
class NMPT_SERVO:
  def __init__(self):
    self.GLOBAL_FLAG = 0
    self.FACES = 0
    self.TOPIC_FACE_TARGET = "/blender_api/set_face_target"
    self.TOPIC_GAZE_TARGET = "/blender_api/set_gaze_target"
    self.TOPIC_SALIENT_POINT = "/nmpt_saliency_point"
    self.TOPIC_FACE_EVENT = "/camera/face_event"
    self.TOPIC_FACE_LOC = "/camera/face_locations"
    
    self.EVENT_NEW_FACE = "new_face"
    self.EVENT_LOST_FACE = "lost_face"
    self.EVENT_TRACK_FACE = "track_face"

    self.salience = rospy.Subscriber(self.TOPIC_SALIENT_POINT, targets, self.lookat_salientP_cb)
    self.look_pub = rospy.Publisher(self.TOPIC_FACE_TARGET, Target, queue_size=10)
    self.look_gaze = rospy.Publisher(self.TOPIC_GAZE_TARGET, Target, queue_size=10)
    rospy.Subscriber(self.TOPIC_FACE_EVENT, FaceEvent, self.face_event_cb)
    rospy.Subscriber(self.TOPIC_FACE_LOC, Faces, self.face_loc_cb)

  # Callback for NMPT_B
  def lookat_salientP_cb(self, data):
    loc = []
    loc = data.positions[0]
    degree = data.degree 
    t = Target()
    t.x = 1.0
    if loc.x <= 0.5:
    	loc.x =(0.5 - loc.x)
    else:
        loc.x =(0.5 -loc.x)
    t.y = loc.x
    if loc.y <= 0.5:
    	loc.y =(0.5 - loc.y)
    else:
        loc.y =(0.5 -loc.y)
    t.z = loc.y

    if self.GLOBAL_FLAG and degree >=7: # Look to Certainly Determined Salient Point
        self.look_pub.publish(t)
        self.look_gaze.publish(t)

  def face_event_cb(self, data):
    if data.face_event == self.EVENT_NEW_FACE:
	print ("Controlling pkg: Face Tracker")
	self.GLOBAL_FLAG = 0
    elif data.face_event == self.EVENT_LOST_FACE:
        print ("Controlling pkg: Salience Detector")
	self.GLOBAL_FLAG = 1
    elif data.face_event == self.EVENT_TRACK_FACE:
	pass

  def face_loc_cb(self, data):
    self.FACES = 0
    for face in data.faces:
    	self.FACES = self.FACES +1
    print self.FACES
    	

if __name__ == '__main__':
    global d
    try:
        rospy.init_node('NMPT_SERVO', anonymous=True)
        NMPT_SERVO()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
