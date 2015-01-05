#
# face_track.py -  Registery and tracking of faces
# Copyright (C) 2014  Hanson Robotics
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

import time

from owyl import blackboard
import rospy
from pi_face_tracker.msg import FaceEvent, Faces

# A Face. Currently consists only of an ID number, a 3D location,
# and the time it was last seen.  Should be extended to include
# the size of the face, possibly the location of the eyes, and,
# if possible, the name of the human attached to it ...
class Face:
	def __init__(self, fid, point):
		self.faceid = fid
		self.x = point.x
		self.y = point.y
		self.z = point.z
		self.t = time.time()


# A registery of all the faces currently visible.
# Copies face data to owyl blackboard.
class FaceTrack:

	def __init__(self, owyl_bboard):

		print("Starting Face Tracker")
		self.blackboard = owyl_bboard
		self.visible_faces = []
		self.face_locations = {}

		# pi_vision topics and events
		self.TOPIC_FACE_EVENT = "face_event"
		self.EVENT_NEW_FACE = "new_face"
		self.EVENT_LOST_FACE = "lost_face"

		self.TOPIC_FACE_LOCATIONS = "face_locations"

		# Face appearance/disappearance from pi_vision
		rospy.Subscriber(self.TOPIC_FACE_EVENT, FaceEvent, self.face_event_cb)

		# Face location information from pi_vision
		rospy.Subscriber(self.TOPIC_FACE_LOCATIONS, Faces, self.face_loc_cb)


	# Add a face to the Owyl blackboard.
	def add_face_to_bb(self, faceid):

		# We already know about it.
		if faceid in self.blackboard["background_face_targets"]:
			return

		# Update the blackboard.
		self.blackboard["is_interruption"] = True
		self.blackboard["new_face"] = faceid
		self.blackboard["background_face_targets"].append(faceid)

	# Remove a face from the Owyl blackboard.
	def remove_face_from_bb(self, fid):

		if fid not in self.blackboard["background_face_targets"]:
			return

		# Update the blackboard.
		self.blackboard["is_interruption"] = True
		self.blackboard["lost_face"] = fid
		self.blackboard["background_face_targets"].remove(fid)
		# If the robot lost the new face during the initial
		# interaction, reset new_face variable
		if self.blackboard["new_face"] == fid :
			self.blackboard["new_face"] = ""

	# Start tracking a face
	def add_face(self, faceid):
		if faceid in self.visible_faces:
			return

		self.visible_faces.append(faceid)

		print "New face added to visibile faces: " + \
			str(self.face_locations.keys()))


	# Stop tracking a face
	def remove_face(self, faceid):
		if faceid in self.visible_faces:
			self.visible_faces.remove(faceid)

		if faceid in self.face_locations:
			del self.face_locations[faceid]

		# print "Lost face; visibile faces now: " + str(self.visible_faces))
		print "Lost face; visibile faces now: " + \
			str(self.face_locations.keys()))


	def face_event_cb(self, data):
		if data.face_event == self.EVENT_NEW_FACE:

			# Keep track of the visible faces.
			self.add_face(data.face_id)
			self.add_face_to_bb(data.face_id)

		elif data.face_event == self.EVENT_LOST_FACE:

			# Keep track of the visible faces.
			self.remove_face(data.face_id)
			self.remove_face_from_bb(data.face_id)

	def face_loc_cb(self, data):
		for face in data.faces:
			fid = face.id
			loc = face.point
			inface = Face(fid, loc)

			# Sanity check.  Sometimes pi_vision sends us faces with
			# location (0,0,0). Discard these.
			if loc.x < 0.05:
				continue

			if fid not in self.visible_faces:
				self.visible_faces.append(fid)
				self.add_face_to_bb(fid)

			self.face_locations[fid] = inface

			# TODO If location has not been reported in a while,
			# remove it from the list. We should have gotten a
			# lost face message for this, but these do not always
			# seem reliable.
			# print("duude ola:" + str(self.face_locations))

